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

from shm_ros.segment import BUCKETS, BUFFER_BASE, CEILING_OFFSET

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


def test_every_bucket_size_is_reachable():
    """A segment sized for any bucket must resolve, or that bucket is dead."""
    from shm_ros.segment import bucket_for_segment

    for ceiling, block_num in BUCKETS:
        assert bucket_for_segment(BUFFER_BASE + block_num * ceiling) is not None
