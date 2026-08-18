/*! @package shm_ros
    Code Information:
        Maintainer: Advanced Robotics Team
        Copyright robot.com / Advanced Robotics Team

    The ROS half of the producer side: owns a SegmentWriter and the ShmImage
    publisher, and does the one thing every producer must get right — write the
    pixels FIRST, announce the block only after.

    A producer node should never open a segment itself. Include this, hand it a
    frame, and the ordering, the ring, the stride announcement, the segment
    naming and the error de-duplication all come with it. Header-only, so there
    is still nothing to link beyond rclcpp and this package's typesupport.
*/

#ifndef SHM_ROS__PUBLISHER_HPP_
#define SHM_ROS__PUBLISHER_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "shm_ros/msg/shm_image.hpp"
#include "shm_ros/segment.hpp"

namespace shm_ros {

// Writes frames into a shared-memory segment and announces them as ShmImage.
class ImagePublisher {
 public:
  // `node` may be an rclcpp::Node or a LifecycleNode. The segment is named after
  // the RESOLVED topic, so it matches what any consumer derives from it.
  template <typename NodeT>
  ImagePublisher(NodeT &node, const std::string &topic, const rclcpp::QoS &qos)
      : logger_(node.get_logger()) {
    publisher_ = node.template create_publisher<msg::ShmImage>(topic, qos);
    topic_name_ = publisher_->get_topic_name();
    segment_name_ = shm_ros::segment_name(topic_name_);
  }

  const std::string &topic_name() const { return topic_name_; }
  const std::string &segment_name() const { return segment_name_; }
  const std::string &segment_path() const { return writer_.path(); }
  const std::string &last_error() const { return last_error_; }
  size_t get_subscription_count() const { return publisher_->get_subscription_count(); }

  // Copy one frame into the ring and announce it. False on failure, with
  // last_error() set; the failure is logged once, not once per frame.
  bool publish(const uint8_t *data, size_t size, uint32_t width, uint32_t height,
               const std::string &encoding, uint32_t step,
               const std_msgs::msg::Header &header) {
    if (data == nullptr || size == 0) return fail("empty frame");
    if (!writer_.open(topic_name_, size)) return fail(writer_.last_error());

    const int64_t block_id = writer_.write(data, size);
    if (block_id < 0) return fail("block write rejected");

    if (!last_error_.empty()) {
      RCLCPP_INFO_STREAM(logger_, "shm_ros: " << topic_name_ << " recovered");
      last_error_.clear();
    }

    msg::ShmImage announcement;
    announcement.header = header;
    // Named explicitly so a consumer never re-derives it — which matters the
    // moment a bridge renames the topic but leaves the segment alone.
    announcement.segment = segment_name_;
    announcement.height = height;
    announcement.width = width;
    announcement.encoding = encoding;
    announcement.step = step;
    announcement.block_id = static_cast<uint64_t>(block_id);
    announcement.size = size;
    announcement.seq = seq_++;
    publisher_->publish(announcement);
    return true;
  }

 private:
  // Log a fault only when it changes, so a stuck producer does not spam.
  bool fail(const std::string &reason) {
    if (reason != last_error_) {
      last_error_ = reason;
      RCLCPP_ERROR_STREAM(logger_, "shm_ros: " << topic_name_ << ": " << reason);
    }
    return false;
  }

  rclcpp::Publisher<msg::ShmImage>::SharedPtr publisher_;
  SegmentWriter writer_;
  rclcpp::Logger logger_;
  std::string topic_name_;
  std::string segment_name_;
  std::string last_error_;
  uint64_t seq_ = 0;
};

}  // namespace shm_ros

#endif  // SHM_ROS__PUBLISHER_HPP_
