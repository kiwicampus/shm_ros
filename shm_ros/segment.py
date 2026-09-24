#!/usr/bin/env python3

# =============================================================================
"""Reader and writer for the shared-memory image transport.

Mirror of ``include/shm_ros/segment.hpp``; see that header for the layout and
why it is shaped this way. Imports no ROS on purpose, so the same code serves
producers, bridges and consumers.
"""

# =============================================================================
import ctypes
import ctypes.util
import mmap
import os
from typing import Optional, Tuple

import numpy as np

# =============================================================================

#: cudaHostRegisterMapped, from cuda_runtime_api.h -- requests a device pointer
#: for the registered range (cudaHostGetDevicePointer only resolves ranges
#: registered with this flag).
_CUDA_HOST_REGISTER_MAPPED = 0x02
_CUDA_MEMCPY_DEVICE_TO_HOST = 2
_CUDA_MEMCPY_DEVICE_TO_DEVICE = 3

_cudart: Optional[ctypes.CDLL] = None
_cudart_load_failed = False


def _load_cudart() -> Optional[ctypes.CDLL]:
    """The CUDA runtime, loaded once. None (cached) if it is not installed.

    ctypes rather than a Python CUDA package (cupy/pycuda/torch) on purpose:
    every shm_ros consumer would otherwise need one of those importable just to
    import ``segment``, even the ones that never touch a GPU frame.
    """
    global _cudart, _cudart_load_failed
    if _cudart is not None or _cudart_load_failed:
        return _cudart
    name = ctypes.util.find_library("cudart") or "libcudart.so"
    try:
        _cudart = ctypes.CDLL(name)
    except OSError:
        _cudart_load_failed = True
        return None
    return _cudart


class CudaHostMapping:
    """ctypes port of ``shm_ros/cuda_host_mapping.hpp``.

    Registers an already-mapped host range with CUDA and hands back the
    device-visible pointer for it (Jetson unified memory, or zero-copy over
    PCIe on a discrete GPU). A no-op producing an unmapped instance when CUDA
    is not installed -- callers check :meth:`is_mapped`.
    """

    def __init__(self) -> None:
        self._host_ptr: Optional[ctypes.c_void_p] = None
        self._device_ptr = 0

    def map(self, address: int, length: int) -> bool:
        """Register ``length`` bytes at ``address``. Idempotent once mapped."""
        if self.is_mapped():
            return True
        cudart = _load_cudart()
        if cudart is None:
            return False

        host_ptr = ctypes.c_void_p(address)
        rc = cudart.cudaHostRegister(
            host_ptr, ctypes.c_size_t(length), ctypes.c_uint(_CUDA_HOST_REGISTER_MAPPED)
        )
        if rc != 0:
            return False

        device_ptr = ctypes.c_void_p()
        rc = cudart.cudaHostGetDevicePointer(
            ctypes.byref(device_ptr), host_ptr, ctypes.c_uint(0)
        )
        if rc != 0:
            cudart.cudaHostUnregister(host_ptr)
            return False

        self._host_ptr = host_ptr
        self._device_ptr = device_ptr.value or 0
        return True

    def is_mapped(self) -> bool:
        """Whether :meth:`map` has succeeded and not since been undone."""
        return self._host_ptr is not None

    def device_ptr(self, offset: int = 0) -> int:
        """Device-visible address of ``offset`` bytes into the mapping, or 0."""
        return self._device_ptr + offset if self.is_mapped() else 0

    def unmap(self) -> None:
        """Undo the registration. Safe to call when never mapped."""
        if self._host_ptr is not None:
            cudart = _load_cudart()
            if cudart is not None:
                cudart.cudaHostUnregister(self._host_ptr)
            self._host_ptr = None
        self._device_ptr = 0


def cuda_memcpy_device_to_host(device_ptr: int, nbytes: int) -> bytes:
    """``nbytes`` read from ``device_ptr`` through the CUDA runtime.

    Round-trips through the GPU exactly as a real kernel consumer would: this
    is what proves ``device_ptr`` is a working, GPU-addressable pointer rather
    than an opaque integer nothing ever dereferenced.
    """
    cudart = _load_cudart()
    if cudart is None:
        raise RuntimeError("CUDA runtime not available")
    buf = ctypes.create_string_buffer(nbytes)
    rc = cudart.cudaMemcpy(
        buf,
        ctypes.c_void_p(device_ptr),
        ctypes.c_size_t(nbytes),
        ctypes.c_int(_CUDA_MEMCPY_DEVICE_TO_HOST),
    )
    if rc != 0:
        raise RuntimeError(f"cudaMemcpy(D2H) failed: rc={rc}")
    return buf.raw


def cuda_memcpy_device_to_device(
    dst_device_ptr: int, src_device_ptr: int, nbytes: int
) -> None:
    """Copy ``nbytes`` device-to-device, entirely on the GPU -- no host involved.

    For a consumer that wants the frame to actually LIVE on the GPU (a torch/
    cupy CUDA tensor it will run a model on) rather than one that just wants
    the bytes: point ``dst_device_ptr`` at memory a GPU framework already
    allocated (e.g. a ``torch.empty(..., device="cuda").data_ptr()``) and the
    shm segment's pixels land there without ever touching this process's RAM.
    """
    cudart = _load_cudart()
    if cudart is None:
        raise RuntimeError("CUDA runtime not available")
    rc = cudart.cudaMemcpy(
        ctypes.c_void_p(dst_device_ptr),
        ctypes.c_void_p(src_device_ptr),
        ctypes.c_size_t(nbytes),
        ctypes.c_int(_CUDA_MEMCPY_DEVICE_TO_DEVICE),
    )
    if rc != 0:
        raise RuntimeError(f"cudaMemcpy(D2D) failed: rc={rc}")


# =============================================================================

#: STATE_SIZE(4096) + EXTRA_SIZE(4096) from astribot's ShmConf. BLOCK_SIZE is 0
#: there, so blocks start at this fixed offset for every bucket.
BUFFER_BASE = 4096 + 4096

#: ``ShmConf::ceiling_msg_size`` in the State record, which starts at offset 0.
#: Written once at creation, so a plain read is enough.
CEILING_OFFSET = 0x18

#: ShmConf's MESSAGE_SIZE_* / BLOCK_NUM_* ladder, ascending.
BUCKETS: Tuple[Tuple[int, int], ...] = (
    (16 << 10, 512),
    (128 << 10, 128),
    (1 << 20, 64),
    (8 << 20, 32),
    (16 << 20, 16),
    (64 << 20, 16),
)


def segment_name(topic: str) -> str:
    """Segment file name for ``topic``: the topic with every '/' removed."""
    return topic.replace("/", "")


def segment_path(topic: str) -> str:
    """Full path of the segment file for ``topic``."""
    return "/dev/shm/" + segment_name(topic)


def channels_for(encoding: str) -> int:
    """BYTES per pixel for an encoding, as in ``sensor_msgs/Image``.

    A channel count is NOT a byte count: mono16 is one channel but two bytes, and
    yuv422_yuy2 is two bytes with no whole number of channels. Unknown encodings
    fall through to 3, which is a guess -- prefer the row stride, exact for every
    format.
    """
    if encoding == "mono8":
        return 1
    if encoding in ("mono16", "yuv422", "yuv422_yuy2", "uyvy", "yuyv", "16UC1"):
        return 2
    if encoding in ("rgba8", "bgra8", "32FC1"):
        return 4
    return 3


def frame_bytes(height: int, width: int, encoding: str) -> int:
    """Size of one frame, guessed from the encoding.

    Fallback only: wrong for any format :func:`channels_for` does not know and for
    any buffer whose rows are padded. Prefer ``height * step``.
    """
    return height * width * channels_for(encoding)


def bucket_for_frame(frame_bytes: int) -> Tuple[int, int]:
    """First bucket a frame fits in, else the largest.

    Mirrors ``ShmConf::GetCeilingMessageSize``, a ``<=`` ladder.
    """
    for ceiling, block_num in BUCKETS:
        if frame_bytes <= ceiling:
            return ceiling, block_num
    return BUCKETS[-1]


def bucket_for_segment(segment_size: int) -> Optional[Tuple[int, int]]:
    """Bucket a segment of this byte size was allocated from, or None.

    Ambiguous by construction: 8 MiB x 32 and 16 MiB x 16 give the same size, so
    first match wins. Prefer the ceiling the writer announces.
    """
    for ceiling, block_num in BUCKETS:
        if BUFFER_BASE + block_num * ceiling == segment_size:
            return ceiling, block_num
    return None


class SegmentReader:
    """Read-only view of one producer's segment. Lazily mapped, cheap per frame."""

    def __init__(self, topic: str) -> None:
        """Record the topic. No I/O until :meth:`open`."""
        self._topic = topic
        self._path = segment_path(topic)
        self._file: Optional[int] = None
        self._map: Optional[mmap.mmap] = None
        self._view: Optional[memoryview] = None
        self._length = 0
        self._stride = 0
        self._block_num = 0
        self._device = -1
        self._inode = -1
        self._cuda_file: Optional[int] = None
        self._cuda_map: Optional[mmap.mmap] = None
        self._cuda_mapping: Optional[CudaHostMapping] = None
        self.last_error = ""

    @property
    def is_ready(self) -> bool:
        """Whether a segment is currently mapped."""
        return self._map is not None

    @property
    def stride(self) -> int:
        """Bytes between block starts, or 0 when not mapped."""
        return self._stride

    @property
    def block_num(self) -> int:
        """Number of blocks in the mapped ring, or 0 when not mapped."""
        return self._block_num

    @property
    def unlinked(self) -> bool:
        """Whether the file we are mapped to has been deleted.

        A producer restart unlinks it; reads then SUCCEED against a dead inode.
        """
        if self._file is None:
            return False
        try:
            return os.fstat(self._file).st_nlink == 0
        except OSError:
            return True

    def open(self) -> bool:
        """Map the segment, re-mapping if the producer replaced it.

        Idempotent and cheap, so call it every frame. Comparing the inode rather
        than the name is the point: a resolution change is a producer restart,
        and a stale mapping would keep serving the old stride.
        """
        try:
            status = os.stat(self._path)
        except OSError as exc:
            self.close()
            self.last_error = f"cannot stat {self._path}: {exc}"
            return False

        if self._map is not None:
            if status.st_dev == self._device and status.st_ino == self._inode:
                return True
            self.close()

        if status.st_size <= BUFFER_BASE:
            self.last_error = f"{self._path}: segment is only {status.st_size} bytes"
            return False

        try:
            descriptor = os.open(self._path, os.O_RDONLY)
        except OSError as exc:
            self.last_error = f"cannot open {self._path}: {exc}"
            return False
        try:
            mapping = mmap.mmap(
                descriptor, status.st_size, mmap.MAP_SHARED, mmap.PROT_READ
            )
        except (OSError, ValueError) as exc:
            os.close(descriptor)
            self.last_error = f"cannot mmap {self._path}: {exc}"
            return False

        self._file = descriptor
        self._map = mapping
        # One long-lived view so per-frame slicing allocates nothing.
        self._view = memoryview(mapping)
        self._length = status.st_size

        geometry = self._resolve_geometry()
        if geometry is None:
            self.close()
            self.last_error = f"{self._path}: size {status.st_size} matches no bucket"
            return False

        self._stride, self._block_num = geometry
        self._device = status.st_dev
        self._inode = status.st_ino
        self.last_error = ""
        return True

    def _resolve_geometry(self) -> Optional[Tuple[int, int]]:
        """Pin the stride for this mapping.

        Size alone is ambiguous: 8 MiB x 32 and 16 MiB x 16 are both 268443648
        bytes. Prefer the bucket the writer names at ``CEILING_OFFSET``, and fall
        back to the size table when it names nothing usable.
        """
        if self._view is not None and self._length >= CEILING_OFFSET + 8:
            announced = int.from_bytes(
                self._view[CEILING_OFFSET : CEILING_OFFSET + 8], "little"
            )
            for ceiling, block_num in BUCKETS:
                if (
                    ceiling == announced
                    and BUFFER_BASE + block_num * ceiling == self._length
                ):
                    return ceiling, block_num
        return bucket_for_segment(self._length)

    def block_offset(self, block_id: int) -> int:
        """Byte offset of ``block_id``, or -1 when it is out of range."""
        if self._map is None or block_id < 0 or block_id >= self._block_num:
            return -1
        return BUFFER_BASE + block_id * self._stride

    def frame(
        self,
        block_id: int,
        height: int,
        width: int,
        channels: int = 3,
        step: int = 0,
    ) -> Optional[np.ndarray]:
        """Read-only view of the frame in ``block_id``, or None.

        ``step`` is bytes per row INCLUDING padding; 0 means rows are packed.
        Aliases the mapping when unpacked: consume it before the writer laps the
        ring. A padded frame is copied, since the slice is not contiguous.
        """
        if self._view is None:
            self.last_error = "segment not mapped"
            return None
        if height <= 0 or width <= 0 or channels <= 0:
            self.last_error = f"bad geometry {width}x{height}x{channels}"
            return None

        row_bytes = width * channels
        stride = step if step > 0 else row_bytes
        if stride < row_bytes:
            # The producer's rows are shorter than width*channels, so `channels`
            # is wrong for this encoding. Fail loudly rather than read garbage.
            self.last_error = (
                f"step {stride} < {width}x{channels}={row_bytes} bytes per row; "
                "the encoding's bytes-per-pixel is not what channels_for returned"
            )
            return None

        offset = self.block_offset(block_id)
        if offset < 0:
            self.last_error = f"block {block_id} outside the ring of {self._block_num}"
            return None

        size = stride * height
        if offset + size > self._length:
            self.last_error = (
                f"block {block_id} needs {size} bytes at {offset}, "
                f"segment is {self._length}"
            )
            return None

        flat = np.frombuffer(self._view[offset : offset + size], dtype=np.uint8)
        if stride == row_bytes:
            return flat.reshape(height, width, channels)
        # Padded rows: reshape by the REAL stride and slice off the padding. A
        # padded buffer reshaped straight to (h, w, c) does not fail — every row
        # after the first starts late and the picture shears, silently.
        return flat.reshape(height, stride)[:, :row_bytes].reshape(
            height, width, channels
        )

    def device_ptr(self, block_id: int, frame_bytes: int) -> int:
        """Same as :meth:`frame`, but a GPU-addressable pointer instead of a
        host view for the caller to copy.

        Lazily opens a SEPARATE read-write mapping of the segment and
        registers THAT with CUDA, on first use: ``cudaHostRegister(...,
        cudaHostRegisterMapped)`` rejects a read-only region outright
        ("invalid argument", confirmed on real hardware), and the normal
        ``map_`` stays read-only on purpose -- a reader must never be able to
        write into someone else's segment, and nothing here needs to write
        through the CUDA mapping either; it exists only so the registration
        call succeeds.

        Returns 0 (not an exception) when CUDA is unavailable, the segment
        isn't mapped, or the geometry is out of range -- callers fall back to
        :meth:`frame`, they don't need to special-case a missing GPU.
        """
        if (
            self._map is None
            or frame_bytes <= 0
            or block_id < 0
            or block_id >= self._block_num
        ):
            return 0
        offset = BUFFER_BASE + block_id * self._stride
        if offset + frame_bytes > self._length:
            self.last_error = (
                f"block {block_id} needs {frame_bytes} bytes at {offset}, "
                f"segment is {self._length}"
            )
            return 0

        if self._cuda_map is None:
            try:
                descriptor = os.open(self._path, os.O_RDWR)
            except OSError as exc:
                self.last_error = f"cannot open {self._path} for CUDA mapping: {exc}"
                return 0
            try:
                mapping = mmap.mmap(
                    descriptor,
                    self._length,
                    mmap.MAP_SHARED,
                    mmap.PROT_READ | mmap.PROT_WRITE,
                )
            except (OSError, ValueError) as exc:
                os.close(descriptor)
                self.last_error = f"cannot mmap {self._path} read-write: {exc}"
                return 0

            buf = (ctypes.c_char * self._length).from_buffer(mapping)
            address = ctypes.addressof(buf)
            cuda_mapping = CudaHostMapping()
            if not cuda_mapping.map(address, self._length):
                mapping.close()
                os.close(descriptor)
                self.last_error = (
                    "cudaHostRegister failed (no CUDA device? see nvidia-smi)"
                )
                return 0

            self._cuda_file = descriptor
            self._cuda_map = mapping
            self._cuda_mapping = cuda_mapping

        assert self._cuda_mapping is not None
        return self._cuda_mapping.device_ptr(offset)

    def close(self) -> None:
        """Unmap and close. Idempotent, and safe with views outstanding."""
        if self._cuda_mapping is not None:
            self._cuda_mapping.unmap()
            self._cuda_mapping = None
        if self._cuda_map is not None:
            try:
                self._cuda_map.close()
            except BufferError:
                pass
            self._cuda_map = None
        if self._cuda_file is not None:
            try:
                os.close(self._cuda_file)
            except OSError:
                pass
            self._cuda_file = None
        if self._view is not None:
            try:
                self._view.release()
            except (BufferError, ValueError):
                # A view is still referenced: drop our handle and let the
                # collector finish, rather than raise out of teardown.
                self.last_error = "close deferred: frame views still referenced"
            self._view = None
        if self._map is not None:
            try:
                self._map.close()
            except BufferError:
                pass
            self._map = None
        if self._file is not None:
            try:
                os.close(self._file)
            except OSError:
                pass
            self._file = None
        self._length = 0
        self._stride = 0
        self._block_num = 0
        self._device = -1
        self._inode = -1

    def __enter__(self) -> "SegmentReader":
        """Map the segment on entry."""
        self.open()
        return self

    def __exit__(self, *_exc_info: object) -> None:
        """Unmap on exit."""
        self.close()


class SegmentWriter:
    """Producer side: owns the segment file and cycles the ring."""

    def __init__(self, topic: str) -> None:
        """Record the topic. Nothing is created until :meth:`open`."""
        self._topic = topic
        self._path = segment_path(topic)
        self._file: Optional[int] = None
        self._map: Optional[mmap.mmap] = None
        self._length = 0
        self._stride = 0
        self._frame_bytes = 0
        self._block_num = 0
        self._seq = 0
        self.last_error = ""

    @property
    def is_ready(self) -> bool:
        """Whether a segment is currently mapped."""
        return self._map is not None

    @property
    def path(self) -> str:
        """Path of the segment file."""
        return self._path

    @property
    def stride(self) -> int:
        """Bytes between block starts, or 0 when not open."""
        return self._stride

    @property
    def block_num(self) -> int:
        """Number of blocks in the ring, or 0 when not open."""
        return self._block_num

    def open(self, frame_bytes: int) -> bool:
        """Create the segment for frames of ``frame_bytes``.

        A no-op when already open with that geometry. Otherwise the old segment
        is unlinked and a NEW inode created: readers detect a restart by inode,
        and resizing in place would leave them on the old stride.
        """
        if frame_bytes <= 0:
            self.last_error = "frame size is zero"
            return False
        if self.is_ready and frame_bytes == self._frame_bytes:
            return True
        self.close()

        name = segment_name(self._topic)
        if not name:
            self.last_error = f"topic '{self._topic}' leaves no segment name"
            return False

        ceiling, block_num = bucket_for_frame(frame_bytes)
        if frame_bytes > ceiling:
            self.last_error = f"frame of {frame_bytes} bytes exceeds every bucket"
            return False
        length = BUFFER_BASE + block_num * ceiling

        try:
            if os.path.exists(self._path):
                os.unlink(self._path)
            descriptor = os.open(self._path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o664)
            os.ftruncate(descriptor, length)
            mapping = mmap.mmap(descriptor, length, mmap.MAP_SHARED, mmap.PROT_WRITE)
        except OSError as exc:
            self.last_error = f"cannot create {self._path}: {exc}"
            return False

        # Publish the stride the way ShmConf does, so a reader never has to guess.
        mapping[0:BUFFER_BASE] = b"\x00" * BUFFER_BASE
        mapping[CEILING_OFFSET : CEILING_OFFSET + 8] = ceiling.to_bytes(8, "little")

        self._file = descriptor
        self._map = mapping
        self._length = length
        self._stride = ceiling
        self._frame_bytes = frame_bytes
        self._block_num = block_num
        self._seq = 0
        self.last_error = ""
        return True

    def write(self, data: bytes) -> int:
        """Copy a frame into the next block and return its id, or -1.

        Announce the block only after this returns.
        """
        if self._map is None or not data or len(data) > self._stride:
            return -1
        block_id = self._seq % self._block_num
        self._seq += 1
        offset = BUFFER_BASE + block_id * self._stride
        self._map[offset : offset + len(data)] = data
        return block_id

    def close(self) -> None:
        """Unmap, close and unlink. Idempotent."""
        if self._map is not None:
            try:
                self._map.close()
            except BufferError:
                pass
            self._map = None
        if self._file is not None:
            try:
                os.close(self._file)
            except OSError:
                pass
            self._file = None
        try:
            if os.path.exists(self._path):
                os.unlink(self._path)
        except OSError:
            pass
        self._length = 0
        self._stride = 0
        self._frame_bytes = 0
        self._block_num = 0
        self._seq = 0

    def __enter__(self) -> "SegmentWriter":
        """Return self; call :meth:`open` with a frame size to create."""
        return self

    def __exit__(self, *_exc_info: object) -> None:
        """Unlink on exit."""
        self.close()


class ImageReader:
    """Consumer-side rules in one place.

    A consumer that drives :class:`SegmentReader` by hand has to remember both of
    these, and both fail SILENTLY when forgotten:

    1. Re-open every frame. A producer restart -- which is what a resolution
       change is -- replaces the segment; a mapping held from before reads the old
       stride at wrong offsets and the picture tears with no error raised.
    2. Honour the segment the producer NAMES over one derived from the topic, so a
       bridge can rename the topic while leaving the segment alone.

    Takes plain fields rather than a message, so it serves any announcement type:
    ``shm_ros/ShmImage`` from our producers, ``astribot_camera/CameraImage`` from
    theirs. Imports no ROS.
    """

    def __init__(self, topic: str) -> None:
        """``topic`` is the fallback segment, until a producer names its own."""
        self._topic = topic
        self._current = ""
        self._reader = SegmentReader(topic)
        self.last_error = ""

    @property
    def reader(self) -> SegmentReader:
        """The underlying reader, for stride and block_num reporting."""
        return self._reader

    @property
    def segment(self) -> str:
        """Segment currently mapped."""
        return self._current

    def frame(
        self,
        block_id: int,
        height: int,
        width: int,
        channels: int = 3,
        segment: str = "",
        step: int = 0,
    ) -> Optional[np.ndarray]:
        """Read-only view of the announced block, or None with ``last_error`` set.

        ``step`` is bytes per row including padding; 0 means packed rows.
        """
        wanted = segment or self._topic
        if wanted != self._current:
            self._reader.close()
            self._reader = SegmentReader(wanted)
            self._current = wanted

        if not self._reader.open():
            self.last_error = self._reader.last_error
            return None

        view = self._reader.frame(block_id, height, width, channels, step)
        if view is None:
            self.last_error = self._reader.last_error
            return None

        self.last_error = ""
        return view

    def frame_from(self, msg: object) -> Optional[np.ndarray]:
        """Same, reading the fields off a ShmImage (or any message shaped like one).

        Passes ``msg.step`` through, so padded rows are sliced rather than
        silently sheared.
        """
        encoding = getattr(msg, "encoding", "") or "rgb8"
        return self.frame(
            msg.block_id,
            msg.height,
            msg.width,
            channels_for(encoding),
            getattr(msg, "segment", "") or "",
            int(getattr(msg, "step", 0) or 0),
        )

    def device_ptr_from(self, msg: object) -> int:
        """GPU-addressable pointer to the announced block, or 0.

        0 means: fall back to :meth:`frame_from`. That covers every reason a
        GPU read isn't available here -- CUDA not installed, the producer
        never stamped ``uses_gpu``, the segment isn't mapped yet -- a caller
        that only wants pixels doesn't need to tell those apart.
        """
        if not getattr(msg, "uses_gpu", False):
            self.last_error = "producer did not stamp uses_gpu"
            return 0

        wanted = getattr(msg, "segment", "") or self._topic
        if wanted != self._current:
            self._reader.close()
            self._reader = SegmentReader(wanted)
            self._current = wanted
        if not self._reader.open():
            self.last_error = self._reader.last_error
            return 0

        encoding = getattr(msg, "encoding", "") or "rgb8"
        step = int(getattr(msg, "step", 0) or 0)
        bytes_needed = (
            step * msg.height
            if step > 0
            else frame_bytes(msg.height, msg.width, encoding)
        )
        ptr = self._reader.device_ptr(msg.block_id, bytes_needed)
        if not ptr:
            self.last_error = self._reader.last_error or "no GPU device pointer"
        return ptr

    def gpu_frame_from(self, msg: object) -> Optional[np.ndarray]:
        """Same picture as :meth:`frame_from`, but read through the GPU.

        Resolves ``device_ptr_from`` and round-trips the bytes with a real
        ``cudaMemcpy(D2H)`` -- the same mechanism a CUDA kernel would use to
        consume the frame, not a stand-in for it. None (with ``last_error``
        set) whenever the GPU path isn't available; callers fall back to
        :meth:`frame_from`.
        """
        ptr = self.device_ptr_from(msg)
        if not ptr:
            return None

        encoding = getattr(msg, "encoding", "") or "rgb8"
        channels = channels_for(encoding)
        row_bytes = msg.width * channels
        step = int(getattr(msg, "step", 0) or 0)
        stride = step if step > 0 else row_bytes
        size = stride * msg.height

        try:
            raw = cuda_memcpy_device_to_host(ptr, size)
        except RuntimeError as exc:
            self.last_error = str(exc)
            return None

        flat = np.frombuffer(raw, dtype=np.uint8)
        if stride == row_bytes:
            return flat.reshape(msg.height, msg.width, channels)
        return flat.reshape(msg.height, stride)[:, :row_bytes].reshape(
            msg.height, msg.width, channels
        )

    def close(self) -> None:
        """Unmap the segment."""
        self._reader.close()
