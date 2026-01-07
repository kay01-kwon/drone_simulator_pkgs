#include "ros_sensor_noise/ros_odom_noise_generator.hpp"
RosOdomNoiseGenerator::RosOdomNoiseGenerator()
{
    node_ = rclcpp::Node::make_shared("ros_odom_noise_generator");

    // Initialize noise parameters (example values)
    noise_params_.pose_noise_stddev = 0.05;
    noise_params_.linear_velocity_noise_stddev = 0.1;
    noise_params_.angle_noise_stddev = 0.02;
    noise_params_.axis_noise_stddev[0] = 0.01;
    noise_params_.axis_noise_stddev[1] = 0.01;
    noise_params_.axis_noise_stddev[2] = 0.01;
    noise_params_.angular_velocity_noise_stddev = 0.05;

    // Create subscriber and publisher
    odom_sub_ = node_->create_subscription<Odometry>(
        "odom", 10, std::bind(&RosOdomNoiseGenerator::odomCallback, this, std::placeholders::_1));

    noisy_odom_pub_ = node_->create_publisher<Odometry>("noisy_odom", 10);

    // Create timer for periodic noise application
    timer_ = node_->create_wall_timer(
        10ms, std::bind(&RosOdomNoiseGenerator::timerCallback, this));
}

RosOdomNoiseGenerator::~RosOdomNoiseGenerator()
{
}

void RosOdomNoiseGenerator::odomCallback(const Odometry::SharedPtr msg)
{
    // Process incoming odometry message
    if (noise_enabled_) {
        applyNoise();
    }
}

void RosOdomNoiseGenerator::applyNoise()
{
    // Apply noise to the odometry data
    // This is a placeholder for the actual noise application logic
}

void RosOdomNoiseGenerator::timerCallback()
{
    // Periodic tasks can be handled here
}

void RosOdomNoiseGenerator::configure()
{
    // Configuration logic for noise parameters can be added here
}