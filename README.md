# shm_ros

Shared-memory image transport for ROS 2. The pixels never go on the wire: a
producer writes a frame into a POSIX shared-memory segment and publishes a small
`ShmImage` announcement naming the block; a consumer maps the segment once and
copies the pixels straight out. Nothing is encoded, nothing is decoded, and DDS
carries about a hundred bytes per frame.

The segment layout is Apollo CyberRT's block transport (Apache-2.0) — the same
one the Astribot camera driver writes — so one reader serves both that driver and
your own producers.

## What is in here

| | |
|---|---|
| `msg/ShmImage.msg` | the announcement: segment, geometry, block id, seq |
| `include/shm_ros/segment.hpp` | header-only `SegmentReader` / `SegmentWriter`. No ROS, no OpenCV |
| `include/shm_ros/publisher.hpp` | header-only `ImagePublisher`: segment + announcement, correctly ordered |
| `shm_ros/segment.py` | the same reader and writer in pure Python |
| `shm_ros/subscriber.py` | `ImageSubscriber`: announcement to numpy frame |

C++ is header-only, so there is nothing to link beyond `rclcpp` and this
package's typesupport.

## Publishing

```cpp
#include <shm_ros/publisher.hpp>

shm_ros::ImagePublisher shm(node, "~/color/image_shm", rclcpp::SensorDataQoS());
shm.publish(data, size, width, height, "rgb8", step, header);
```

## Consuming

```python
from shm_ros.subscriber import ImageSubscriber

def on_frame(frame, msg):      # frame is an HxWxC numpy view of the block
    ...

source = ImageSubscriber(node, "/camera/color/image_shm", on_frame)
```

```cpp
#include <shm_ros/segment.hpp>

shm_ros::SegmentReader reader;
if (reader.open(msg->segment.empty() ? topic : msg->segment)) {
  const uint8_t *pixels = reader.frame(msg->block_id, msg->height * msg->width * 3);
}
```

## Two rules that fail silently if you break them

Use `ImagePublisher` and `ImageSubscriber` and you get both for free. If you
drive `SegmentReader` and `SegmentWriter` yourself:

1. **A consumer must re-open every frame.** `open()` is idempotent and cheap, and
   it compares the inode. A producer restart — which is what a resolution change
   is — replaces the segment. A mapping held from before keeps reading the old
   stride at the wrong offsets, and no error is raised. The picture just tears.
2. **A producer must write the pixels before announcing the block.** That
   ordering is the whole reason reads need no lock.

## How it works

```
segment file : /dev/shm/<topic with '/' stripped>
State record : at offset 0; the stride (ceiling_msg_size) sits at 0x18
buffer       : starts at 8192 = STATE_SIZE(4096) + EXTRA_SIZE(4096)
block stride : the bucket ceiling — 16K, 128K, 1M, 8M, 16M or 64M
segment size : 8192 + block_num * ceiling, exactly
```

Reads are lock-free. A frame is announced only after the writer releases the
block, and the writer walks the whole ring before reusing one — at 30 Hz with 32
blocks that is over a second of grace against a copy measured in milliseconds.

**Read the stride from `0x18`, never infer it from the file size.** Segment sizes
are ambiguous: 8 MiB x 32 and 16 MiB x 16 are both 268443648 bytes. Guessing
picks the wrong stride half the time and shears every frame.

## Building

```bash
colcon build --packages-select shm_ros
colcon test --packages-select shm_ros
```

The C++ header and the Python module each carry their own copy of the layout
constants, which is a drift hazard. `test/test_layout_parity.py` parses the
header and asserts the two agree, so a change to one that misses the other fails
the build.

## ROS 1

`shm_ros` also builds under catkin, message only, so a Noetic node can publish
`ShmImage` across `ros1_bridge`. `mapping_rules.yaml` forces the package pair,
which the bridge needs because the name does not end in `_msgs`.

## Licence

Apache-2.0.
