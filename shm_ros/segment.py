#!/usr/bin/env python3

# =============================================================================
"""Reader and writer for the shared-memory image transport.

Mirror of ``include/shm_ros/segment.hpp``; see that header for the layout and
why it is shaped this way. Imports no ROS on purpose, so the same code serves
producers, bridges and consumers.
"""

# =============================================================================
import mmap
import os
from typing import Optional, Tuple

import numpy as np

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
        self, block_id: int, height: int, width: int, channels: int = 3
    ) -> Optional[np.ndarray]:
        """Read-only view of the frame in ``block_id``, or None.

        Aliases the mapping: consume it before the writer laps the ring.
        """
        if self._view is None:
            self.last_error = "segment not mapped"
            return None
        if height <= 0 or width <= 0 or channels <= 0:
            self.last_error = f"bad geometry {width}x{height}x{channels}"
            return None

        offset = self.block_offset(block_id)
        if offset < 0:
            self.last_error = f"block {block_id} outside the ring of {self._block_num}"
            return None

        size = height * width * channels
        if offset + size > self._length:
            self.last_error = (
                f"block {block_id} needs {size} bytes at {offset}, "
                f"segment is {self._length}"
            )
            return None

        return np.frombuffer(
            self._view[offset : offset + size], dtype=np.uint8
        ).reshape(height, width, channels)

    def close(self) -> None:
        """Unmap and close. Idempotent, and safe with views outstanding."""
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

    #: Channels per pixel by encoding; anything unlisted is treated as 3.
    CHANNELS = {"mono8": 1, "mono16": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}

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
    ) -> Optional[np.ndarray]:
        """Read-only view of the announced block, or None with ``last_error`` set."""
        wanted = segment or self._topic
        if wanted != self._current:
            self._reader.close()
            self._reader = SegmentReader(wanted)
            self._current = wanted

        if not self._reader.open():
            self.last_error = self._reader.last_error
            return None

        view = self._reader.frame(block_id, height, width, channels)
        if view is None:
            self.last_error = self._reader.last_error
            return None

        self.last_error = ""
        return view

    def frame_from(self, msg: object) -> Optional[np.ndarray]:
        """Same, reading the fields off a ShmImage (or any message shaped like one)."""
        encoding = getattr(msg, "encoding", "") or "rgb8"
        return self.frame(
            msg.block_id,
            msg.height,
            msg.width,
            self.CHANNELS.get(encoding, 3),
            getattr(msg, "segment", "") or "",
        )

    def close(self) -> None:
        """Unmap the segment."""
        self._reader.close()
