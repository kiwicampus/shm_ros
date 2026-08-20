#!/usr/bin/env python3
"""Super simple live viewer for a shm_ros stream. Usage:

    ros2 run shm_ros shm_ros_viewer.py [topic]   # default: /video_mapping/webcamtest/image_raw
                                                  # (subscribes to <topic>/shm)
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from shm_ros.msg import ShmImage
from shm_ros.segment import ImageReader

_REINTERPRET_DTYPE = {"16UC1": np.uint16, "32FC1": np.float32}


class ShmViewer(Node):
    def __init__(self, topic):
        super().__init__("shm_ros_viewer")
        self.reader = ImageReader(topic)
        self.count = 0
        self.create_subscription(
            ShmImage, topic + "/shm", self.on_frame, qos_profile_sensor_data
        )
        self.get_logger().info(f"Watching {topic}/shm")

    def on_frame(self, msg):
        image = self.reader.frame_from(msg)
        if image is None:
            self.get_logger().warning(f"dropped frame: {self.reader.last_error}")
            return

        encoding = msg.encoding or "rgb8"
        dtype = _REINTERPRET_DTYPE.get(encoding)
        if dtype is not None:
            image = image.view(dtype).reshape(image.shape[:2])
        elif image.shape[2] == 1:
            image = image[..., 0]

        self.count += 1
        if encoding == "rgb8":
            frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif dtype is not None:
            # depth (16UC1 in mm, or 32FC1 in meters) -- normalize just for display
            frame = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        elif image.ndim == 2:
            frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            frame = image

        cv2.putText(
            frame,
            f"seq={msg.seq} block={msg.block_id} #{self.count} {msg.width}x{msg.height} {encoding}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.imshow("shm_ros_viewer", frame)
        cv2.waitKey(1)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/video_mapping/webcamtest/image_raw"
    rclpy.init()
    node = ShmViewer(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
