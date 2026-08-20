# Announcement for one image frame living in POSIX shared memory.
#
# The pixels are NOT in this message. They sit in a block of the segment named by
# `segment`, and this message says which block and how to interpret it. Readers
# mmap the segment once and copy the announced block out; nothing is decoded and
# nothing travels over DDS but these few bytes.
#
# The layout is Apollo CyberRT's block transport, the same one the astribot
# camera driver writes, so a reader serves both producers. See shm_ros/segment.hpp.

std_msgs/Header header

# Segment file, without the /dev/shm prefix. Empty means "derive it from the
# topic this was published on, with every '/' removed" — the driver's own rule.
# Set it when a bridge renames the topic but not the segment.
string segment

uint32 height
uint32 width
string encoding             # rgb8, bgr8, mono8, ... as in sensor_msgs/Image
uint32 step                 # bytes per row, padding included

uint64 block_id             # index of the block holding this frame
uint64 size                 # payload bytes written into that block
uint64 seq                  # producer frame counter, for gap detection

# Producer-side kill switch for the GPU path (Jetson unified memory via
# cudaHostRegister -- see include/shm_ros/cuda_host_mapping.hpp). Not required
# for the CUDA mechanics themselves: a consumer could call device_ptr() either
# way. It lets an operator disable every consumer's GPU attempt from one
# producer param, without touching consumer code.
bool uses_gpu
