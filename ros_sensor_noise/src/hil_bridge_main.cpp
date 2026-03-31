#include "ros_sensor_noise/gz_hil_bridge.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto bridge = std::make_shared<GzHilBridge>();

    rclcpp::spin(bridge->getNode());

    rclcpp::shutdown();
    return 0;
}
