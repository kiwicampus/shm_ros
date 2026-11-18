#include "shm_to_image.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<shm_ros::ShmToImageNode>());
    rclcpp::shutdown();
    return 0;
}
