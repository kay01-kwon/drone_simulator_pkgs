#include "ros_sensor_noise/ros_odom_noise_generator.hpp"
#include <cmath>

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
    node_->declare_parameter("noise.linear_velocity_stddev_x", 0.01);
    node_->declare_parameter("noise.linear_velocity_stddev_y", 0.01);
    node_->declare_parameter("noise.linear_velocity_stddev_z", 0.01);
    node_->declare_parameter("noise.angular_velocity_stddev_x", 0.01);
    node_->declare_parameter("noise.angular_velocity_stddev_y", 0.01);
    node_->declare_parameter("noise.angular_velocity_stddev_z", 0.01);
    node_->declare_parameter("noise.apply_noise", true);

    // Declare position offset parameter
    node_->declare_parameter("noise.z_offset", 0.0);

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
    noise_params_.linear_velocity_noise_stddev[0] = node_->get_parameter("noise.linear_velocity_stddev_x").as_double();
    noise_params_.linear_velocity_noise_stddev[1] = node_->get_parameter("noise.linear_velocity_stddev_y").as_double();
    noise_params_.linear_velocity_noise_stddev[2] = node_->get_parameter("noise.linear_velocity_stddev_z").as_double();
    noise_params_.angular_velocity_noise_stddev[0] = node_->get_parameter("noise.angular_velocity_stddev_x").as_double();
    noise_params_.angular_velocity_noise_stddev[1] = node_->get_parameter("noise.angular_velocity_stddev_y").as_double();
    noise_params_.angular_velocity_noise_stddev[2] = node_->get_parameter("noise.angular_velocity_stddev_z").as_double();
    noise_enabled_ = node_->get_parameter("noise.apply_noise").as_bool();

    // Load position offset
    z_offset_ = node_->get_parameter("noise.z_offset").as_double();

    RCLCPP_INFO(node_->get_logger(), "=== Topic Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Odometry input topic: %s", odom_input_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Odometry output topic: %s", odom_output_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Pose output topic: %s", pose_output_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "=== Noise Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Noise enabled: %s", noise_enabled_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Position noise stddev: %.6f", noise_params_.pose_noise_stddev);
    RCLCPP_INFO(node_->get_logger(), "Linear velocity noise stddev: [%.6f, %.6f, %.6f]",
        noise_params_.linear_velocity_noise_stddev[0],
        noise_params_.linear_velocity_noise_stddev[1],
        noise_params_.linear_velocity_noise_stddev[2]);
    RCLCPP_INFO(node_->get_logger(), "Angle noise stddev: %.6f", noise_params_.angle_noise_stddev);
    RCLCPP_INFO(node_->get_logger(), "Axis noise stddev: %.6f", axis_stddev);
    RCLCPP_INFO(node_->get_logger(), "Angular velocity noise stddev: [%.6f, %.6f, %.6f]",
        noise_params_.angular_velocity_noise_stddev[0],
        noise_params_.angular_velocity_noise_stddev[1],
        noise_params_.angular_velocity_noise_stddev[2]);
    RCLCPP_INFO(node_->get_logger(), "=== Offset Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Z offset: %.4f m", z_offset_);

    // Create subscriber and publishers
    odom_sub_ = node_->create_subscription<Odometry>(
        odom_input_topic, rclcpp::SensorDataQoS(),
        std::bind(&RosOdomNoiseGenerator::odomCallback,
                  this, std::placeholders::_1));

    noisy_odom_pub_ = node_->create_publisher<Odometry>(
        odom_output_topic, rclcpp::SensorDataQoS());
    noisy_pose_pub_ = node_->create_publisher<PoseStamped>(
        pose_output_topic, 10);

    // Initialize random number generator
    std::random_device rd;
    gen_ = std::mt19937(rd());
}

RosOdomNoiseGenerator::~RosOdomNoiseGenerator()
{
}

void RosOdomNoiseGenerator::odomCallback(const Odometry::SharedPtr msg)
{
    // Apply position & velocity offset (base_link → imu_link)
    // Rigid body: v_P = v_O + ω × r, computed in world frame
    Odometry offset_odom = *msg;
    if (std::abs(z_offset_) > 1e-6) {
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        Eigen::Vector3d offset_body(0.0, 0.0, z_offset_);
        Eigen::Vector3d offset_world = q * offset_body;
        offset_odom.pose.pose.position.x += offset_world.x();
        offset_odom.pose.pose.position.y += offset_world.y();
        offset_odom.pose.pose.position.z += offset_world.z();

        Eigen::Vector3d omega_body(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);
        Eigen::Vector3d omega_world = q * omega_body;
        Eigen::Vector3d dv_world = omega_world.cross(offset_world);
        offset_odom.twist.twist.linear.x += dv_world.x();
        offset_odom.twist.twist.linear.y += dv_world.y();
        offset_odom.twist.twist.linear.z += dv_world.z();
    }

    // Apply noise (or pass through)
    Odometry noisy_odom;
    if (noise_enabled_) {
        noisy_odom = applyNoise(offset_odom);
    } else {
        noisy_odom = offset_odom;
        noisy_odom.header.stamp = node_->now();
    }

    noisy_odom_pub_->publish(noisy_odom);

    PoseStamped pose;
    pose.header = noisy_odom.header;
    pose.pose = noisy_odom.pose.pose;
    noisy_pose_pub_->publish(pose);
}

Odometry RosOdomNoiseGenerator::applyNoise(const Odometry& odom)
{
    Odometry noisy_odom = odom;

    // Apply noise to position (x, y, z)
    std::normal_distribution<double> pos_noise(
        0.0, noise_params_.pose_noise_stddev);
    noisy_odom.pose.pose.position.x += pos_noise(gen_);
    noisy_odom.pose.pose.position.y += pos_noise(gen_);
    noisy_odom.pose.pose.position.z += pos_noise(gen_);

    // Apply per-axis noise to linear velocity
    std::normal_distribution<double> lin_vel_noise_x(0.0, noise_params_.linear_velocity_noise_stddev[0]);
    std::normal_distribution<double> lin_vel_noise_y(0.0, noise_params_.linear_velocity_noise_stddev[1]);
    std::normal_distribution<double> lin_vel_noise_z(0.0, noise_params_.linear_velocity_noise_stddev[2]);
    noisy_odom.twist.twist.linear.x += lin_vel_noise_x(gen_);
    noisy_odom.twist.twist.linear.y += lin_vel_noise_y(gen_);
    noisy_odom.twist.twist.linear.z += lin_vel_noise_z(gen_);

    // Apply noise to quaternion using angle-axis representation
    Eigen::Quaterniond quat(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z
    );

    Eigen::AngleAxisd angle_axis(quat);
    double angle = angle_axis.angle();
    Eigen::Vector3d axis = angle_axis.axis();

    std::normal_distribution<double> angle_noise(
        0.0, noise_params_.angle_noise_stddev);
    angle += angle_noise(gen_);

    std::normal_distribution<double> axis_noise_x(
        0.0, noise_params_.axis_noise_stddev[0]);
    std::normal_distribution<double> axis_noise_y(
        0.0, noise_params_.axis_noise_stddev[1]);
    std::normal_distribution<double> axis_noise_z(
        0.0, noise_params_.axis_noise_stddev[2]);
    axis.x() += axis_noise_x(gen_);
    axis.y() += axis_noise_y(gen_);
    axis.z() += axis_noise_z(gen_);

    axis.normalize();

    Eigen::AngleAxisd noisy_angle_axis(angle, axis);
    Eigen::Quaterniond noisy_quat(noisy_angle_axis);

    noisy_odom.header.stamp = node_->now();

    noisy_odom.pose.pose.orientation.w = noisy_quat.w();
    noisy_odom.pose.pose.orientation.x = noisy_quat.x();
    noisy_odom.pose.pose.orientation.y = noisy_quat.y();
    noisy_odom.pose.pose.orientation.z = noisy_quat.z();

    // Apply per-axis noise to angular velocity
    std::normal_distribution<double> ang_vel_noise_x(0.0, noise_params_.angular_velocity_noise_stddev[0]);
    std::normal_distribution<double> ang_vel_noise_y(0.0, noise_params_.angular_velocity_noise_stddev[1]);
    std::normal_distribution<double> ang_vel_noise_z(0.0, noise_params_.angular_velocity_noise_stddev[2]);
    noisy_odom.twist.twist.angular.x += ang_vel_noise_x(gen_);
    noisy_odom.twist.twist.angular.y += ang_vel_noise_y(gen_);
    noisy_odom.twist.twist.angular.z += ang_vel_noise_z(gen_);

    return noisy_odom;
}

