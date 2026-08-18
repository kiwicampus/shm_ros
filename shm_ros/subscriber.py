#!/usr/bin/env python3

# =============================================================================
"""The ROS half of the consumer side.

A consumer should never drive a :class:`SegmentReader` by hand: the two rules
that matter are easy to get wrong and fail silently when you do.

1. Re-open EVERY frame. A producer restart — which is what a resolution change
   is — replaces the segment, and a mapping held from before keeps reading the
   old stride at wrong offsets. No error is raised; the picture just tears.
2. Trust the segment the producer names over any derived from the topic, since
   a bridge can rename the topic while leaving the segment alone.

:class:`ImageSubscriber` does both.
"""

# =============================================================================
from typing import Callable, Optional

import numpy as np

from shm_ros.segment import SegmentReader

# =============================================================================

#: Channels per pixel, by encoding. Anything else is treated as 3.
CHANNELS = {"mono8": 1, "mono16": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}


class ImageSubscriber:
    """Turns ShmImage announcements into numpy frames.

    Pass ``node`` and a topic and it subscribes for you, or use
    :meth:`frame_from` directly if you already own the subscription.
    """

    def __init__(
        self,
        node: object,
        topic: str,
        callback: Optional[Callable[[np.ndarray, object], None]] = None,
        qos: int = 10,
        segment: str = "",
    ) -> None:
        """Subscribe to ``topic``; ``callback`` gets (frame, msg) per frame.

        ``segment`` overrides the name only until the producer announces its own.
        """
        # Imported here, not at module scope, so shm_ros.segment stays usable in
        # plain Python (bridges, tools, tests) with no ROS on the path.
        from shm_ros.msg import ShmImage

        self._callback = callback
        self._segment = segment or topic
        self._reader = SegmentReader(self._segment)
        self.last_error = ""
        self.subscription = node.create_subscription(ShmImage, topic, self._on_message, qos)

    @property
    def reader(self) -> SegmentReader:
        """The underlying reader, for stride/block_num reporting."""
        return self._reader

    def frame_from(self, msg: object) -> Optional[np.ndarray]:
        """Read the block ``msg`` announces, or None with ``last_error`` set."""
        segment = getattr(msg, "segment", "") or self._segment
        if segment != self._segment:
            self._segment = segment
            self._reader.close()
            self._reader = SegmentReader(segment)

        # Every frame: see rule 1 in the module docstring.
        if not self._reader.open():
            self.last_error = self._reader.last_error
            return None

        channels = CHANNELS.get(msg.encoding, 3)
        frame = self._reader.frame(msg.block_id, msg.height, msg.width, channels)
        if frame is None:
            self.last_error = self._reader.last_error
            return None

        self.last_error = ""
        return frame

    def _on_message(self, msg: object) -> None:
        """Hand a decoded frame to the callback, if there is one."""
        frame = self.frame_from(msg)
        if frame is not None and self._callback is not None:
            self._callback(frame, msg)

    def close(self) -> None:
        """Unmap the segment. The subscription is the node's to destroy."""
        self._reader.close()
