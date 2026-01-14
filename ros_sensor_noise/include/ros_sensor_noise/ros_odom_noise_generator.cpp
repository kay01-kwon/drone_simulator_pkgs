#include "ros_sensor_noise/ros_odom_noise_generator.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

RosOdomNoiseGenerator::RosOdomNoiseGenerator()
{
    node_ = rclcpp::Node::make_shared("ros_odom_noise_generator");

    // Declare topic parameters
    node_->declare_parameter("topics.odom_input", "odom");
    node_->declare_parameter("topics.odom_output", "noisy_odom");
    node_->declare_parameter("topics.pose_output", "noisy_pose");

    // Declare noise parameters
    node_->declare_parameter("noise.position_stddev", 0.002);
    node_->declare_parameter("noise.angle_stddev", 0.01);
    node_->declare_parameter("noise.axis_stddev", 0.005);
    node_->declare_parameter("noise.linear_velocity_stddev", 0.01);
    node_->declare_parameter("noise.angular_velocity_stddev", 0.01);
    node_->declare_parameter("noise.apply_noise", true);

    // Load topic parameters
    std::string odom_input_topic = node_->get_parameter("topics.odom_input").as_string();
    std::string odom_output_topic = node_->get_parameter("topics.odom_output").as_string();
    std::string pose_output_topic = node_->get_parameter("topics.pose_output").as_string();

    // Load noise parameters from config
    noise_params_.pose_noise_stddev = node_->get_parameter("noise.position_stddev").as_double();
    noise_params_.angle_noise_stddev = node_->get_parameter("noise.angle_stddev").as_double();
    double axis_stddev = node_->get_parameter("noise.axis_stddev").as_double();
    noise_params_.axis_noise_stddev[0] = axis_stddev;
    noise_params_.axis_noise_stddev[1] = axis_stddev;
    noise_params_.axis_noise_stddev[2] = axis_stddev;
    noise_params_.linear_velocity_noise_stddev = node_->get_parameter("noise.linear_velocity_stddev").as_double();
    noise_params_.angular_velocity_noise_stddev = node_->get_parameter("noise.angular_velocity_stddev").as_double();
    noise_enabled_ = node_->get_parameter("noise.apply_noise").as_bool();

    RCLCPP_INFO(node_->get_logger(), "=== Topic Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Odometry input topic: %s", odom_input_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Odometry output topic: %s", odom_output_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Pose output topic: %s", pose_output_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "=== Noise Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Noise enabled: %s", noise_enabled_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Position noise stddev: %.6f", noise_params_.pose_noise_stddev);
    RCLCPP_INFO(node_->get_logger(), "Linear velocity noise stddev: %.6f", noise_params_.linear_velocity_noise_stddev);
    RCLCPP_INFO(node_->get_logger(), "Angle noise stddev: %.6f", noise_params_.angle_noise_stddev);
    RCLCPP_INFO(node_->get_logger(), "Axis noise stddev: %.6f", axis_stddev);
    RCLCPP_INFO(node_->get_logger(), "Angular velocity noise stddev: %.6f", noise_params_.angular_velocity_noise_stddev);

    // Create subscriber and publishers
    odom_sub_ = node_->create_subscription<Odometry>(
        odom_input_topic, rclcpp::SensorDataQoS(), std::bind(&RosOdomNoiseGenerator::odomCallback, this, std::placeholders::_1));

    noisy_odom_pub_ = node_->create_publisher<Odometry>(odom_output_topic, rclcpp::SensorDataQoS());
    noisy_pose_pub_ = node_->create_publisher<PoseStamped>(pose_output_topic, rclcpp::SensorDataQoS());

    // Initialize random number generator
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

RosOdomNoiseGenerator::~RosOdomNoiseGenerator()
{
}

void RosOdomNoiseGenerator::odomCallback(const Odometry::SharedPtr msg)
{
    // Store the incoming odometry message
    latest_odom_ = *msg;

    // Process incoming odometry message
    if (noise_enabled_) {
        applyNoise();
    } else {
        // If noise is disabled, just republish the original messages
        msg->header.frame_id = "noise_disabled";
        msg->header.stamp = node_->now();

        noisy_odom_pub_->publish(*msg);

        PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        noisy_pose_pub_->publish(pose);
    }
}

void RosOdomNoiseGenerator::applyNoise()
{
    // Create noisy odometry message
    Odometry noisy_odom = latest_odom_;

    // Apply noise to position (x, y, z)
    std::normal_distribution<double> pos_noise(0.0, noise_params_.pose_noise_stddev);
    noisy_odom.pose.pose.position.x += pos_noise(gen_);
    noisy_odom.pose.pose.position.y += pos_noise(gen_);
    noisy_odom.pose.pose.position.z += pos_noise(gen_);

    // Apply noise to linear velocity
    std::normal_distribution<double> lin_vel_noise(0.0, noise_params_.linear_velocity_noise_stddev);
    noisy_odom.twist.twist.linear.x += lin_vel_noise(gen_);
    noisy_odom.twist.twist.linear.y += lin_vel_noise(gen_);
    noisy_odom.twist.twist.linear.z += lin_vel_noise(gen_);

    // Apply noise to quaternion using angle-axis representation
    // 1. Convert quaternion to Eigen quaternion
    Eigen::Quaterniond quat(
        latest_odom_.pose.pose.orientation.w,
        latest_odom_.pose.pose.orientation.x,
        latest_odom_.pose.pose.orientation.y,
        latest_odom_.pose.pose.orientation.z
    );

    // 2. Convert to angle-axis
    Eigen::AngleAxisd angle_axis(quat);
    double angle = angle_axis.angle();
    Eigen::Vector3d axis = angle_axis.axis();

    // 3. Apply noise to angle
    std::normal_distribution<double> angle_noise(0.0, noise_params_.angle_noise_stddev);
    angle += angle_noise(gen_);

    // 4. Apply noise to axis
    std::normal_distribution<double> axis_noise_x(0.0, noise_params_.axis_noise_stddev[0]);
    std::normal_distribution<double> axis_noise_y(0.0, noise_params_.axis_noise_stddev[1]);
    std::normal_distribution<double> axis_noise_z(0.0, noise_params_.axis_noise_stddev[2]);
    axis.x() += axis_noise_x(gen_);
    axis.y() += axis_noise_y(gen_);
    axis.z() += axis_noise_z(gen_);

    // 5. Normalize the axis
    axis.normalize();

    // 6. Create new angle-axis with noisy values
    Eigen::AngleAxisd noisy_angle_axis(angle, axis);

    // 7. Convert back to quaternion
    Eigen::Quaterniond noisy_quat(noisy_angle_axis);

    noisy_odom.header.stamp = node_->now();

    // 8. Set the noisy quaternion in the message
    noisy_odom.pose.pose.orientation.w = noisy_quat.w();
    noisy_odom.pose.pose.orientation.x = noisy_quat.x();
    noisy_odom.pose.pose.orientation.y = noisy_quat.y();
    noisy_odom.pose.pose.orientation.z = noisy_quat.z();

    // Apply noise to angular velocity
    std::normal_distribution<double> ang_vel_noise(0.0, noise_params_.angular_velocity_noise_stddev);
    noisy_odom.twist.twist.angular.x += ang_vel_noise(gen_);
    noisy_odom.twist.twist.angular.y += ang_vel_noise(gen_);
    noisy_odom.twist.twist.angular.z += ang_vel_noise(gen_);

    // Publish the noisy odometry
    noisy_odom_pub_->publish(noisy_odom);

    // Create and publish noisy pose
    PoseStamped noisy_pose;
    noisy_pose.header.stamp = node_->now();
    noisy_pose.pose = noisy_odom.pose.pose;
    noisy_pose_pub_->publish(noisy_pose);
}

void RosOdomNoiseGenerator::timerCallback()
{
    // Periodic tasks can be handled here if needed
}

void RosOdomNoiseGenerator::configure()
{
    // Configuration logic for noise parameters can be added here
}