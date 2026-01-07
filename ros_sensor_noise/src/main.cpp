#include "ros_sensor_noise/ros_odom_noise_generator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto noise_generator = std::make_shared<RosOdomNoiseGenerator>();

    rclcpp::spin(noise_generator->getNode());

    rclcpp::shutdown();
    return 0;
}
