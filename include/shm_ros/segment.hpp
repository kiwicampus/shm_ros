/*! @package shm_ros
    Code Information:
        Maintainer: Advanced Robotics Team
        Copyright robot.com / Advanced Robotics Team

    Header-only reader and writer for the POSIX shared-memory transport that
    carries camera frames between processes on one host. The ROS message is only
    an announcement — a block id and geometry — so a consumer copies pixels
    straight out of the segment with no decode and nothing on the wire.

    The layout is Apollo CyberRT's block transport (Apache-2.0), the same one the
    astribot camera driver writes. Every constant below was read out of that
    driver's own symbols (ShmConf in libastribot_camera_lib.so) and verified
    against a running device, so our producers and theirs share one reader:

      segment file  : /dev/shm/<topic with '/' stripped>
      State record  : at offset 0; ceiling_msg_size (the stride) lives at 0x18
      buffer region : starts at STATE_SIZE(4096) + EXTRA_SIZE(4096); BLOCK_SIZE is 0
      block stride  : the bucket ceiling of the frame (16K/128K/1M/8M/16M/64M)
      segment size  : kBufferBase + block_num * ceiling, exactly

    Reads are lock-free. A frame is announced only AFTER the writer releases the
    block, and the writer walks every block before reusing one — at 30 Hz with 32
    blocks that is over a second of grace against a ~1 ms copy. The Block locks in
    the vendor library are process-local and protect nothing across processes, so
    there is no cross-process lock to honour here.

    This header pulls in no ROS and no OpenCV on purpose: the same code serves the
    driver side, the bridge side and the consumer side.
*/

#ifndef SHM_ROS__SEGMENT_HPP_
#define SHM_ROS__SEGMENT_HPP_

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace shm_ros {

// STATE_SIZE(4096) + EXTRA_SIZE(4096) from astribot's ShmConf. BLOCK_SIZE is 0
// there, so blocks start at this fixed offset for every bucket.
inline constexpr size_t kBufferBase = 4096 + 4096;

// ShmConf::ceiling_msg_size inside the State record, which starts at offset 0.
// Written once at creation, so a plain aligned load is enough.
// (0x00 vptr, 0x08 need_remap, 0x0c seq, 0x14 reference_counts, 0x18 ceiling.)
inline constexpr size_t kCeilingOffset = 0x18;

// ShmConf's MESSAGE_SIZE_* / BLOCK_NUM_* ladder, ascending.
struct Bucket {
  size_t ceiling;
  uint32_t block_num;
};
inline constexpr Bucket kBuckets[] = {
    {16ull << 10, 512}, {128ull << 10, 128}, {1ull << 20, 64},
    {8ull << 20, 32},   {16ull << 20, 16},   {64ull << 20, 16},
};
inline constexpr size_t kBucketCount = sizeof(kBuckets) / sizeof(kBuckets[0]);

// The driver keys the segment file by the topic with every '/' removed.
inline std::string segment_name(const std::string &topic) {
  std::string name;
  name.reserve(topic.size());
  for (char character : topic) {
    if (character != '/') name += character;
  }
  return name;
}

inline std::string segment_path(const std::string &topic) {
  return "/dev/shm/" + segment_name(topic);
}

// ShmConf::GetCeilingMessageSize: the first bucket that fits, else the largest.
inline const Bucket &bucket_for_frame(size_t frame_bytes) {
  for (const Bucket &bucket : kBuckets) {
    if (frame_bytes <= bucket.ceiling) return bucket;
  }
  return kBuckets[kBucketCount - 1];
}

// ============================================================================

// Read-only view of one producer's segment. Lazily mapped, cheap per frame.
class SegmentReader {
 public:
  SegmentReader() = default;
  ~SegmentReader() { close(); }

  SegmentReader(const SegmentReader &) = delete;
  SegmentReader &operator=(const SegmentReader &) = delete;

  bool is_ready() const { return map_ != nullptr; }
  size_t stride() const { return stride_; }
  uint32_t block_num() const { return block_num_; }
  const std::string &last_error() const { return last_error_; }

  // Map the segment for `topic`. Idempotent, and safe to call every frame: it
  // re-maps when the producer restarted and replaced the file. Comparing the
  // inode rather than the name is the whole point — a stale mapping keeps
  // serving pre-restart pixels forever, and a resolution change IS a restart.
  bool open(const std::string &topic) {
    const std::string path = segment_path(topic);

    struct stat live {};
    if (::stat(path.c_str(), &live) != 0) {
      close();
      last_error_ = "cannot stat " + path + ": " + std::strerror(errno);
      return false;
    }
    if (map_ != nullptr && topic == topic_ && live.st_dev == dev_ &&
        live.st_ino == ino_) {
      return true;
    }
    close();

    if (static_cast<size_t>(live.st_size) <= kBufferBase) {
      last_error_ = path + ": segment is only " + std::to_string(live.st_size) + " bytes";
      return false;
    }

    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
      last_error_ = "cannot open " + path + ": " + std::strerror(errno);
      return false;
    }
    void *map = mmap(nullptr, static_cast<size_t>(live.st_size), PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
      ::close(fd);
      last_error_ = "cannot mmap " + path + ": " + std::strerror(errno);
      return false;
    }

    fd_ = fd;
    map_ = map;
    len_ = static_cast<size_t>(live.st_size);
    dev_ = live.st_dev;
    ino_ = live.st_ino;
    topic_ = topic;

    if (!resolve_geometry()) {
      last_error_ = path + ": size " + std::to_string(len_) + " matches no bucket";
      close();
      return false;
    }
    last_error_.clear();
    return true;
  }

  // Whether the file we are mapped to has been deleted. A producer restart
  // unlinks it, and reads then keep SUCCEEDING against the dead inode.
  bool unlinked() const {
    if (fd_ < 0) return false;
    struct stat self {};
    return fstat(fd_, &self) != 0 || self.st_nlink == 0;
  }

  // Pointer to `frame_bytes` of pixels in `block_id`, or nullptr. The pointer
  // aliases the mapping: copy it out before the writer laps the ring.
  const uint8_t *frame(uint64_t block_id, size_t frame_bytes) const {
    if (map_ == nullptr || frame_bytes == 0 || block_id >= block_num_) return nullptr;
    const size_t offset = kBufferBase + static_cast<size_t>(block_id) * stride_;
    if (offset + frame_bytes > len_) return nullptr;
    return static_cast<const uint8_t *>(map_) + offset;
  }

  void close() {
    if (map_ != nullptr) {
      munmap(map_, len_);
      map_ = nullptr;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    len_ = 0;
    stride_ = 0;
    block_num_ = 0;
    dev_ = 0;
    ino_ = 0;
    topic_.clear();
  }

 private:
  // Pin the stride, among the buckets whose size matches the segment. Size alone
  // is ambiguous — 8 MiB x 32 and 16 MiB x 16 are both 268443648 bytes — so
  // prefer the one the writer names at kCeilingOffset, and fall back to the first
  // match when it names nothing usable (a producer predating the field).
  bool resolve_geometry() {
    uint64_t announced = 0;
    if (len_ >= kCeilingOffset + sizeof(announced)) {
      std::memcpy(&announced, static_cast<uint8_t *>(map_) + kCeilingOffset, sizeof(announced));
    }
    const Bucket *pick = nullptr;
    for (const Bucket &bucket : kBuckets) {
      if (kBufferBase + static_cast<size_t>(bucket.block_num) * bucket.ceiling != len_) continue;
      if (bucket.ceiling == announced) {
        pick = &bucket;
        break;
      }
      if (pick == nullptr) pick = &bucket;
    }
    if (pick == nullptr) return false;
    stride_ = pick->ceiling;
    block_num_ = pick->block_num;
    return true;
  }

  int fd_ = -1;
  void *map_ = nullptr;
  size_t len_ = 0;
  size_t stride_ = 0;
  uint32_t block_num_ = 0;
  dev_t dev_ = 0;
  ino_t ino_ = 0;
  std::string topic_;
  mutable std::string last_error_;
};

// ============================================================================

// Producer side: owns the segment file and cycles the ring.
class SegmentWriter {
 public:
  SegmentWriter() = default;
  ~SegmentWriter() { close(); }

  SegmentWriter(const SegmentWriter &) = delete;
  SegmentWriter &operator=(const SegmentWriter &) = delete;

  bool is_ready() const { return map_ != nullptr; }
  const std::string &path() const { return path_; }
  size_t stride() const { return stride_; }
  uint32_t block_num() const { return block_num_; }
  const std::string &last_error() const { return last_error_; }

  // Size and create the segment for `topic`, holding frames of `frame_bytes`.
  // A no-op when already open with that geometry. Otherwise the old segment is
  // unlinked and a NEW inode created: readers detect a restart by inode, and
  // resizing in place would leave them reading the old stride forever.
  bool open(const std::string &topic, size_t frame_bytes) {
    if (frame_bytes == 0) {
      last_error_ = "frame size is zero";
      return false;
    }
    if (is_ready() && topic == topic_ && frame_bytes == frame_bytes_) return true;
    close();

    const std::string name = segment_name(topic);
    if (name.empty()) {
      last_error_ = "topic '" + topic + "' leaves no segment name";
      return false;
    }
    const Bucket &bucket = bucket_for_frame(frame_bytes);
    if (frame_bytes > bucket.ceiling) {
      last_error_ = "frame of " + std::to_string(frame_bytes) + " bytes exceeds every bucket";
      return false;
    }
    const size_t length = kBufferBase + static_cast<size_t>(bucket.block_num) * bucket.ceiling;

    shm_name_ = "/" + name;
    shm_unlink(shm_name_.c_str());
    const int fd = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0664);
    if (fd < 0) {
      last_error_ = "shm_open /dev/shm/" + name + " failed: " + std::strerror(errno);
      shm_name_.clear();
      return false;
    }
    if (ftruncate(fd, static_cast<off_t>(length)) != 0) {
      last_error_ = "ftruncate failed: " + std::string(std::strerror(errno));
      ::close(fd);
      shm_unlink(shm_name_.c_str());
      shm_name_.clear();
      return false;
    }
    void *map = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
      last_error_ = "mmap failed: " + std::string(std::strerror(errno));
      ::close(fd);
      shm_unlink(shm_name_.c_str());
      shm_name_.clear();
      return false;
    }

    // Publish the stride the way ShmConf does. Without it a reader must guess
    // from the file size, which cannot tell 8 MiB x 32 from 16 MiB x 16.
    std::memset(map, 0, kBufferBase);
    const uint64_t ceiling = static_cast<uint64_t>(bucket.ceiling);
    std::memcpy(static_cast<uint8_t *>(map) + kCeilingOffset, &ceiling, sizeof(ceiling));
    std::atomic_thread_fence(std::memory_order_release);

    topic_ = topic;
    path_ = "/dev/shm/" + name;
    fd_ = fd;
    map_ = map;
    len_ = length;
    stride_ = bucket.ceiling;
    frame_bytes_ = frame_bytes;
    block_num_ = bucket.block_num;
    seq_ = 0;
    last_error_.clear();
    return true;
  }

  // Copy one frame into the next block and return its id, or -1. Announce the
  // block only after this returns: that ordering is what makes reads safe.
  int64_t write(const uint8_t *data, size_t size) {
    if (map_ == nullptr || data == nullptr || size == 0 || size > stride_) return -1;
    const uint32_t block_id = static_cast<uint32_t>(seq_++ % block_num_);
    std::memcpy(static_cast<uint8_t *>(map_) + kBufferBase + block_id * stride_, data, size);
    std::atomic_thread_fence(std::memory_order_release);
    return static_cast<int64_t>(block_id);
  }

  void close() {
    if (map_ != nullptr) {
      munmap(map_, len_);
      map_ = nullptr;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    if (!shm_name_.empty()) {
      shm_unlink(shm_name_.c_str());
      shm_name_.clear();
    }
    topic_.clear();
    path_.clear();
    len_ = 0;
    stride_ = 0;
    frame_bytes_ = 0;
    block_num_ = 0;
    seq_ = 0;
  }

 private:
  std::string topic_;
  std::string path_;
  std::string shm_name_;
  int fd_ = -1;
  void *map_ = nullptr;
  size_t len_ = 0;
  size_t stride_ = 0;
  size_t frame_bytes_ = 0;
  uint32_t block_num_ = 0;
  uint64_t seq_ = 0;
  std::string last_error_;
};

}  // namespace shm_ros

#endif  // SHM_ROS__SEGMENT_HPP_
