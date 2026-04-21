#include "ros_sensor_noise/att_odom_publisher.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
