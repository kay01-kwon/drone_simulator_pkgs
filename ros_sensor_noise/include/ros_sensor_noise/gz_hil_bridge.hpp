#ifndef GZ_HIL_BRIDGE_HPP
#define GZ_HIL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/hil_sensor.hpp>
#include <cmath>

using sensor_msgs::msg::Imu;
using nav_msgs::msg::Odometry;
using mavros_msgs::msg::HilSensor;

class GzHilBridge
{
public:
    GzHilBridge();
    ~GzHilBridge() = default;

    rclcpp::Node::SharedPtr getNode() { return node_; }

private:
    void imuCallback(const Imu::SharedPtr msg);
    void odomCallback(const Odometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<HilSensor>::SharedPtr hil_sensor_pub_;

    // Latest altitude from odom (for barometer simulation)
    double latest_alt_{0.0};

    // Magnetic field in NED frame (Gauss) - approximate Earth field
    double mag_north_{0.21};
    double mag_east_{0.0};
    double mag_down_{0.42};
};

#endif // GZ_HIL_BRIDGE_HPP
