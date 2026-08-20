#!/usr/bin/env python3
"""Bridges a plain sensor_msgs/Image topic (e.g. from `ros2 bag play`) into a
shm_ros segment, so a recorded rosbag -- or any node that only publishes plain
Image -- can feed a use_shm=true consumer without its own shm producer.
Mirrors what shm_ros::ImagePublisher does in C++, for the one case that
library has no Python equivalent for: a plain Image message we didn't
capture ourselves.

Usage:
    ros2 run shm_ros image_to_shm_bridge.py <image_topic> [image_topic2 ...]

Then `ros2 bag play <bag>` and subscribe to `<image_topic>/shm`.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from shm_ros.msg import ShmImage
from shm_ros.segment import SegmentWriter, segment_name


class ImageToShmBridge:
    """One topic's bridge: owns the writer, the announcement publisher, and
    the frame counter -- everything shm_ros::ImagePublisher bundles in C++.
    """

    def __init__(self, node: Node, topic: str):
        self.topic = topic
        shm_topic = topic + "/shm"
        self.writer = SegmentWriter(shm_topic)
        self.segment = segment_name(shm_topic)
        self.seq = 0
        self.pub = node.create_publisher(ShmImage, shm_topic, qos_profile_sensor_data)
        node.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg: Image) -> None:
        data = bytes(msg.data)
        if not self.writer.open(len(data)):
            return
        block_id = self.writer.write(data)
        if block_id < 0:
            return

        out = ShmImage()
        out.header = msg.header
        out.segment = self.segment
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.step = msg.step
        out.block_id = block_id
        out.size = len(data)
        out.seq = self.seq
        out.uses_gpu = False
        self.seq += 1
        self.pub.publish(out)


def main():
    topics = sys.argv[1:]
    if not topics:
        print(__doc__)
        raise SystemExit(1)

    rclpy.init()
    node = Node("image_to_shm_bridge")
    bridges = [ImageToShmBridge(node, topic) for topic in topics]
    for bridge in bridges:
        node.get_logger().info(f"Bridging {bridge.topic} -> {bridge.topic}/shm")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
