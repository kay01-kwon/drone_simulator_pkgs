#ifndef ROS_ODOM_NOISE_GENERATOR_HPP
#define ROS_ODOM_NOISE_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

struct NoiseParameters
{
    double pose_noise_stddev;
    double linear_velocity_noise_stddev[3];
    double angle_noise_stddev;
    double axis_noise_stddev[3];
    double angular_velocity_noise_stddev[3];
};

class RosOdomNoiseGenerator
{
    public:

    RosOdomNoiseGenerator();

    ~RosOdomNoiseGenerator();

    rclcpp::Node::SharedPtr getNode() { return node_; }

    private:

    void odomCallback(const Odometry::SharedPtr msg);

    Odometry applyNoise(const Odometry& odom);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr noisy_odom_pub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr noisy_pose_pub_;

    // Noise parameters
    bool noise_enabled_{false};
    NoiseParameters noise_params_;

    // Random number generator
    std::mt19937 gen_;

    // Position offset (e.g., IMU mount offset from base_link)
    double z_offset_{0.0};

};

#endif  // ROS_ODOM_NOISE_GENERATOR_HPP
