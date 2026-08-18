#!/usr/bin/env python3

# =============================================================================
"""Round-trip tests for the Python reader and writer."""

# =============================================================================
import numpy as np
import pytest

from shm_ros.segment import (
    BUCKETS,
    BUFFER_BASE,
    CEILING_OFFSET,
    SegmentReader,
    SegmentWriter,
    bucket_for_frame,
    bucket_for_segment,
    segment_name,
    segment_path,
)

# =============================================================================

TOPIC = "/shm_ros_test/color/image_shm"


@pytest.fixture
def writer():
    """A writer on a test topic, always unlinked afterwards."""
    handle = SegmentWriter(TOPIC)
    yield handle
    handle.close()


def test_segment_name_strips_every_slash():
    assert segment_name("/astribot_camera/head_rgbd/color_image") == (
        "astribot_camerahead_rgbdcolor_image"
    )
    assert segment_path("/a/b") == "/dev/shm/ab"


@pytest.mark.parametrize("frame,expected", [(1, 16 << 10), (16 << 10, 16 << 10),
                                            ((16 << 10) + 1, 128 << 10),
                                            (640 * 480 * 3, 1 << 20),
                                            (1280 * 720 * 3, 8 << 20),
                                            (1 + (16 << 20), 64 << 20)])
def test_bucket_ladder_is_a_less_or_equal_ladder(frame, expected):
    assert bucket_for_frame(frame)[0] == expected


def test_round_trip(writer):
    """Pixels written come back byte-identical through a separate reader."""
    height, width = 480, 640
    frame = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
    assert writer.open(frame.nbytes)
    block_id = writer.write(frame.tobytes())
    assert block_id == 0

    reader = SegmentReader(TOPIC)
    assert reader.open(), reader.last_error
    assert np.array_equal(reader.frame(block_id, height, width), frame)
    reader.close()


def test_writer_announces_its_stride(writer):
    """The ceiling at 0x18 is what lets a reader skip the ambiguous size table."""
    assert writer.open(640 * 480 * 3)
    with open(writer.path, "rb") as handle:
        handle.seek(CEILING_OFFSET)
        announced = int.from_bytes(handle.read(8), "little")
    assert announced == writer.stride == (1 << 20)


def test_reader_follows_the_announced_stride(writer):
    """8 MiB x 32 and 16 MiB x 16 share a segment size; only 0x18 separates them."""
    assert BUFFER_BASE + 32 * (8 << 20) == BUFFER_BASE + 16 * (16 << 20)
    assert writer.open(1280 * 720 * 3)
    reader = SegmentReader(TOPIC)
    assert reader.open(), reader.last_error
    assert (reader.stride, reader.block_num) == (8 << 20, 32)
    reader.close()


def test_ring_wraps_and_reuses_blocks(writer):
    """The writer must walk the whole ring before reusing a block."""
    assert writer.open(1024)
    ring = writer.block_num
    seen = [writer.write(b"\x01" * 1024) for _ in range(ring + 1)]
    assert seen[:ring] == list(range(ring))
    assert seen[ring] == 0


def test_reader_rejects_a_block_outside_the_ring(writer):
    assert writer.open(1024)
    reader = SegmentReader(TOPIC)
    assert reader.open(), reader.last_error
    assert reader.frame(reader.block_num, 1, 1, 1) is None
    reader.close()


def test_reader_notices_a_producer_restart(writer):
    """A geometry change unlinks and recreates; the reader must follow the inode."""
    assert writer.open(640 * 480 * 3)
    reader = SegmentReader(TOPIC)
    assert reader.open(), reader.last_error
    assert reader.stride == (1 << 20)

    assert writer.open(1280 * 720 * 3)  # new inode, new bucket
    assert reader.unlinked
    assert reader.open(), reader.last_error
    assert reader.stride == (8 << 20)
    reader.close()


def test_unknown_segment_size_matches_no_bucket():
    assert bucket_for_segment(BUFFER_BASE + 12345) is None


@pytest.mark.parametrize("ceiling,block_num", BUCKETS)
def test_writer_geometry_matches_the_table(ceiling, block_num):
    assert bucket_for_frame(ceiling) == (ceiling, block_num)
