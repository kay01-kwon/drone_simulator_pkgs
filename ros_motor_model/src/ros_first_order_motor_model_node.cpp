#include "ros_motor_model/ros_first_order_motor_model.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosFirstOrderMotorModelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}