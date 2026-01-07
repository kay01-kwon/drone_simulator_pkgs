#ifndef ROS_ODOM_NOISE_GENERATOR_HPP
#define ROS_ODOM_NOISE_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Pose;

struct NoiseParameters
{
    double pose_noise_stddev;
    double linear_velocity_noise_stddev;
    double angle_noise_stddev;
    double axis_noise_stddev[3];
    double angular_velocity_noise_stddev;
};

class RosOdomNoiseGenerator
{
    public:

    RosOdomNoiseGenerator();

    ~RosOdomNoiseGenerator();

    private:

    void odomCallback(const Odometry::SharedPtr msg);

    void applyNoise();

    void timerCallback();

    void configure();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr noisy_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    // Noise parameters
    bool noise_enabled_{false};
    NoiseParameters noise_params_;

};

#endif  // ROS_ODOM_NOISE_GENERATOR_HPP