#include "ros_sensor_noise/ros_odom_delay_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto delay_node = std::make_shared<RosOdomDelayNode>();

    rclcpp::spin(delay_node->getNode());

    rclcpp::shutdown();
    return 0;
}
