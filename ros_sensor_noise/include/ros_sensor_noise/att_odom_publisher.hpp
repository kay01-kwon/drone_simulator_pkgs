#ifndef ATT_ODOM_PUBLISHER_HPP
#define ATT_ODOM_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <unordered_map>

class AttOdomPublisher : public rclcpp::Node
{
public:
    AttOdomPublisher();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double model_origin_z_;
    double pivot_offset_z_;
    double imu_offset_z_;

    std::string odom_frame_;
    std::string child_frame_;
};

#endif
