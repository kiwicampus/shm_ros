#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "shm_to_image.hpp"

namespace {

const char kTopic[] = "/shm_ros_test/depth_image_shm";

class ShmImageConverterTest : public ::testing::Test
{
   protected:
    // A 4x3 mono16 frame with distinct bytes, written into a fresh segment.
    void SetUp() override
    {
        for (size_t i = 0; i < kBytes; ++i) frame_.push_back(static_cast<uint8_t>(i));
        ASSERT_TRUE(writer_.open(kTopic, kBytes)) << writer_.last_error();
        block_id_ = writer_.write(frame_.data(), kBytes);
        ASSERT_GE(block_id_, 0);
    }

    shm_ros::msg::ShmImage announcement(uint32_t step) const
    {
        shm_ros::msg::ShmImage msg;
        msg.header.stamp.sec = 42;
        msg.header.frame_id = "producer_frame";
        msg.height = kHeight;
        msg.width = kWidth;
        msg.encoding = "mono16";
        msg.step = step;
        msg.block_id = static_cast<uint64_t>(block_id_);
        msg.size = kBytes;
        return msg;
    }

    static constexpr uint32_t kWidth = 4;
    static constexpr uint32_t kHeight = 3;
    static constexpr size_t kBytes = kWidth * kHeight * 2;
    shm_ros::SegmentWriter writer_;
    std::vector<uint8_t> frame_;
    int64_t block_id_ = -1;
};

TEST_F(ShmImageConverterTest, CopiesFrameAndAppliesOverrides)
{
    shm_ros::ShmImageConverter converter({"depth", kTopic, "/out", "camera_depth", "16UC1"});
    sensor_msgs::msg::Image image;
    ASSERT_TRUE(converter.convert(announcement(kWidth * 2), image)) << converter.last_error();

    EXPECT_EQ(image.header.stamp.sec, 42);
    EXPECT_EQ(image.header.frame_id, "camera_depth");
    EXPECT_EQ(image.encoding, "16UC1");
    EXPECT_EQ(image.width, kWidth);
    EXPECT_EQ(image.height, kHeight);
    EXPECT_EQ(image.step, kWidth * 2);
    EXPECT_EQ(image.data, frame_);
}

TEST_F(ShmImageConverterTest, KeepsProducerFieldsAndDerivesStepWhenUnset)
{
    shm_ros::ShmImageConverter converter({"depth", kTopic, "/out", "", ""});
    sensor_msgs::msg::Image image;
    ASSERT_TRUE(converter.convert(announcement(0), image)) << converter.last_error();

    EXPECT_EQ(image.header.frame_id, "producer_frame");
    EXPECT_EQ(image.encoding, "mono16");
    EXPECT_EQ(image.step, kWidth * 2);
    EXPECT_EQ(image.data, frame_);
}

TEST_F(ShmImageConverterTest, FailsOnBlockOutsideTheRing)
{
    shm_ros::ShmImageConverter converter({"depth", kTopic, "/out", "", ""});
    auto msg = announcement(0);
    msg.block_id = 1u << 20;
    sensor_msgs::msg::Image image;
    EXPECT_FALSE(converter.convert(msg, image));
    EXPECT_FALSE(converter.last_error().empty());
}

TEST_F(ShmImageConverterTest, FailsWhenSegmentIsMissing)
{
    shm_ros::ShmImageConverter converter({"depth", "/shm_ros_test/missing", "/out", "", ""});
    auto msg = announcement(0);
    msg.segment = "";
    sensor_msgs::msg::Image image;
    EXPECT_FALSE(converter.convert(msg, image));
}

}  // namespace
