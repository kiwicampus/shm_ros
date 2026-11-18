/*! @package shm_ros
    Republishes ShmImage announcements as plain sensor_msgs/Image, one stream per configured topic,
    so stock tools such as ros2 bag record and rviz can consume shared memory frames.
*/

#ifndef SHM_ROS__SHM_TO_IMAGE_HPP_
#define SHM_ROS__SHM_TO_IMAGE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "shm_ros/msg/shm_image.hpp"
#include "shm_ros/segment.hpp"

namespace shm_ros {

struct StreamConfig
{
    std::string name;
    std::string input_topic;
    std::string output_topic;
    // Empty keeps the producer's value.
    std::string frame_id;
    std::string encoding;
};

// Copies the frame an announcement names out of shared memory into an Image.
class ShmImageConverter
{
   public:
    explicit ShmImageConverter(StreamConfig config);

    // False with last_error() set when the block cannot be read.
    bool convert(const msg::ShmImage& msg, sensor_msgs::msg::Image& image);

    const StreamConfig& config() const { return config_; }
    const std::string& last_error() const { return last_error_; }

   private:
    StreamConfig config_;
    ImageReader reader_;
    std::string last_error_;
};

class ShmToImageNode : public rclcpp::Node
{
   public:
    explicit ShmToImageNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    struct Stream
    {
        explicit Stream(StreamConfig config) : converter(std::move(config)) {}
        ShmImageConverter converter;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
        rclcpp::Subscription<msg::ShmImage>::SharedPtr subscription;
    };

    StreamConfig declare_stream(const std::string& name);
    void on_frame(Stream& stream, const msg::ShmImage& msg);

    std::vector<std::unique_ptr<Stream>> streams_;
};

}  // namespace shm_ros

#endif  // SHM_ROS__SHM_TO_IMAGE_HPP_
