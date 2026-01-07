#include "ros_motor_model/ros_second_order_motor_model.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosSecondOrderMotorModelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}