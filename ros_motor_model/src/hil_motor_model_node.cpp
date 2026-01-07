#include "ros_motor_model/hil_motor_model.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HilMotorModelNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}