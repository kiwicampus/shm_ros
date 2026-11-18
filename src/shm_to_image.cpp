#include "shm_to_image.hpp"

#include <utility>

namespace shm_ros {

ShmImageConverter::ShmImageConverter(StreamConfig config) : config_(std::move(config)), reader_(config_.input_topic) {}

bool ShmImageConverter::convert(const msg::ShmImage& msg, sensor_msgs::msg::Image& image)
{
    const uint32_t step = msg.step > 0 ? msg.step : static_cast<uint32_t>(msg.width * channels_for(msg.encoding));
    const size_t bytes = frame_bytes_from_step(msg.height, step);
    const uint8_t* pixels = reader_.frame(msg.block_id, bytes, msg.segment);
    if (pixels == nullptr)
    {
        last_error_ = reader_.last_error();
        return false;
    }

    image.header.stamp = msg.header.stamp;
    image.header.frame_id = config_.frame_id.empty() ? msg.header.frame_id : config_.frame_id;
    image.height = msg.height;
    image.width = msg.width;
    image.encoding = config_.encoding.empty() ? msg.encoding : config_.encoding;
    image.is_bigendian = 0;
    image.step = step;
    image.data.assign(pixels, pixels + bytes);
    last_error_.clear();
    return true;
}

ShmToImageNode::ShmToImageNode(const rclcpp::NodeOptions& options) : rclcpp::Node("shm_to_image", options)
{
    const auto names = declare_parameter<std::vector<std::string>>("streams", std::vector<std::string>{});
    const auto output_depth = declare_parameter<int>("output_qos_depth", 5);
    if (names.empty())
    {
        RCLCPP_WARN(get_logger(), "no streams configured");
    }

    for (const auto& name : names)
    {
        auto stream = std::make_unique<Stream>(declare_stream(name));
        const auto& config = stream->converter.config();
        // Reliable output so a recorder does not lose whole frames to fragment drops.
        stream->publisher = create_publisher<sensor_msgs::msg::Image>(
            config.output_topic, rclcpp::QoS(static_cast<size_t>(output_depth)).reliable());
        Stream* raw = stream.get();
        stream->subscription =
            create_subscription<msg::ShmImage>(config.input_topic, rclcpp::SensorDataQoS(),
                                               [this, raw](const msg::ShmImage& msg) { on_frame(*raw, msg); });
        RCLCPP_INFO(get_logger(), "%s: %s -> %s", name.c_str(), config.input_topic.c_str(),
                    config.output_topic.c_str());
        streams_.push_back(std::move(stream));
    }
}

StreamConfig ShmToImageNode::declare_stream(const std::string& name)
{
    StreamConfig config;
    config.name = name;
    config.input_topic = declare_parameter<std::string>(name + ".input_topic");
    config.output_topic = declare_parameter<std::string>(name + ".output_topic");
    config.frame_id = declare_parameter<std::string>(name + ".frame_id", "");
    config.encoding = declare_parameter<std::string>(name + ".encoding", "");
    return config;
}

void ShmToImageNode::on_frame(Stream& stream, const msg::ShmImage& msg)
{
    // Copying a frame nobody reads is the whole cost of this node, so skip it.
    if (stream.publisher->get_subscription_count() == 0 &&
        stream.publisher->get_intra_process_subscription_count() == 0)
    {
        return;
    }
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    if (!stream.converter.convert(msg, *image))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: %s", stream.converter.config().name.c_str(),
                             stream.converter.last_error().c_str());
        return;
    }
    stream.publisher->publish(std::move(image));
}

}  // namespace shm_ros
