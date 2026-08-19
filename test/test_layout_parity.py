#!/usr/bin/env python3

# =============================================================================
"""The C++ header and the Python module must agree on the layout.

Both halves carry their own copy of the ShmConf ladder, and a producer that
disagrees with its consumer by one bucket writes frames nobody can read. This
parses the constants straight out of segment.hpp and compares.
"""

# =============================================================================
import re
from pathlib import Path

from shm_ros.segment import BUCKETS, BUFFER_BASE, CEILING_OFFSET, channels_for

# =============================================================================

HEADER = Path(__file__).resolve().parent.parent / "include" / "shm_ros" / "segment.hpp"


def _header() -> str:
    """Source of the C++ core."""
    return HEADER.read_text()


def _shift(text: str) -> int:
    """Evaluate the '16ull << 10' style literals the header uses."""
    match = re.fullmatch(r"\s*(\d+)ull\s*<<\s*(\d+)\s*", text)
    assert match, f"unparsed literal: {text!r}"
    return int(match.group(1)) << int(match.group(2))


def test_header_is_where_we_think():
    assert HEADER.is_file(), f"missing {HEADER}"


def test_buffer_base_matches():
    match = re.search(r"kBufferBase\s*=\s*(\d+)\s*\+\s*(\d+)\s*;", _header())
    assert match, "kBufferBase not found in the header"
    assert int(match.group(1)) + int(match.group(2)) == BUFFER_BASE


def test_ceiling_offset_matches():
    match = re.search(r"kCeilingOffset\s*=\s*(0x[0-9a-fA-F]+)\s*;", _header())
    assert match, "kCeilingOffset not found in the header"
    assert int(match.group(1), 16) == CEILING_OFFSET


def test_bucket_ladder_matches_exactly():
    """Same buckets, same values, same ORDER — first match wins on a tie."""
    block = re.search(r"kBuckets\[\]\s*=\s*\{(.*?)\};", _header(), re.S)
    assert block, "kBuckets not found in the header"
    pairs = re.findall(r"\{([^,]+),\s*(\d+)\}", block.group(1))
    from_header = tuple((_shift(ceiling), int(blocks)) for ceiling, blocks in pairs)
    assert from_header == BUCKETS


def test_channels_per_encoding_match():
    """The two channel tables must agree, or a mono producer tears in one language."""
    body = re.search(r"inline size_t channels_for\(.*?\n\}", _header(), re.S)
    assert body, "channels_for not found in the header"

    # Flatten first: conditions wrap across lines to stay inside the line limit,
    # and a line-by-line parser silently skips them — checking nothing.
    flat = " ".join(body.group(0).split())

    from_header = {}
    for condition, value in re.findall(r"if \((.*?)\)\s*\{?\s*return\s*(\d+);", flat):
        for encoding in re.findall(r'"([^"]+)"', condition):
            from_header[encoding] = int(value)
    default = re.search(r"return\s*(\d+);\s*\}$", flat)
    assert default, "channels_for has no default"

    assert from_header, "no encodings parsed out of channels_for"
    for encoding, channels in from_header.items():
        assert channels_for(encoding) == channels, encoding
    # And the fall-through, for anything the table does not name.
    assert channels_for("something_new") == int(default.group(1))
    # The 2-byte formats must actually be in there — this is the class of bug
    # that shipped: a 2-byte encoding silently sized as 3.
    assert from_header.get("yuv422_yuy2") == 2
    assert from_header.get("mono16") == 2


def test_frame_bytes_uses_the_channel_table():
    from shm_ros.segment import frame_bytes

    assert frame_bytes(480, 640, "rgb8") == 480 * 640 * 3
    assert frame_bytes(480, 640, "mono8") == 480 * 640
    assert frame_bytes(480, 640, "rgba8") == 480 * 640 * 4


def test_every_bucket_size_is_reachable():
    """A segment sized for any bucket must resolve, or that bucket is dead."""
    from shm_ros.segment import bucket_for_segment

    for ceiling, block_num in BUCKETS:
        assert bucket_for_segment(BUFFER_BASE + block_num * ceiling) is not None
