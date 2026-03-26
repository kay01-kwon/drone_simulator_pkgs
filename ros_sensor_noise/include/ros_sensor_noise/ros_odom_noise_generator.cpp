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
    node_->declare_parameter("noise.linear_velocity_stddev", 0.01);
    node_->declare_parameter("noise.angular_velocity_stddev", 0.01);
    node_->declare_parameter("noise.apply_noise", true);

    // Declare delay parameter
    node_->declare_parameter("noise.delay_ms", 0.0);

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

    // Load delay parameter
    delay_ms_ = node_->get_parameter("noise.delay_ms").as_double();
    delay_ns_ = static_cast<int64_t>(delay_ms_ * 1e6);

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
    RCLCPP_INFO(node_->get_logger(), "=== Delay Configuration ===");
    RCLCPP_INFO(node_->get_logger(), "Pure delay: %.1f ms", delay_ms_);

    // Create subscriber and publishers
    odom_sub_ = node_->create_subscription<Odometry>(
        odom_input_topic, rclcpp::SensorDataQoS(), std::bind(&RosOdomNoiseGenerator::odomCallback, this, std::placeholders::_1));

    noisy_odom_pub_ = node_->create_publisher<Odometry>(odom_output_topic, rclcpp::SensorDataQoS());
    noisy_pose_pub_ = node_->create_publisher<PoseStamped>(pose_output_topic, rclcpp::SensorDataQoS());

    // Initialize random number generator
    std::random_device rd;
    gen_ = std::mt19937(rd());

    // If delay is enabled, create a timer to publish delayed messages
    if (delay_ns_ > 0)
    {
        delay_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&RosOdomNoiseGenerator::delayTimerCallback, this));
    }
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

        if (delay_ns_ > 0) {
            // Buffer for delayed publishing
            noisy_odom_buffer_.push_back(*msg);
        } else {
            noisy_odom_pub_->publish(*msg);

            PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->pose.pose;
            noisy_pose_pub_->publish(pose);
        }
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

    if (delay_ns_ > 0) {
        // Buffer the noisy odom for delayed publishing
        noisy_odom_buffer_.push_back(noisy_odom);

        // Prune old messages
        auto latest_stamp = rclcpp::Time(noisy_odom.header.stamp);
        while (!noisy_odom_buffer_.empty())
        {
            auto oldest_stamp = rclcpp::Time(noisy_odom_buffer_.front().header.stamp);
            if ((latest_stamp - oldest_stamp).nanoseconds() > MAX_BUFFER_DURATION_NS)
                noisy_odom_buffer_.pop_front();
            else
                break;
        }
    } else {
        // No delay: publish immediately
        noisy_odom_pub_->publish(noisy_odom);

        PoseStamped noisy_pose;
        noisy_pose.header.stamp = node_->now();
        noisy_pose.pose = noisy_odom.pose.pose;
        noisy_pose_pub_->publish(noisy_pose);
    }
}

void RosOdomNoiseGenerator::delayTimerCallback()
{
    publishDelayed();
}

void RosOdomNoiseGenerator::publishDelayed()
{
    if (noisy_odom_buffer_.size() < 2)
        return;

    // Target time = latest buffered time - delay
    auto latest_stamp = rclcpp::Time(noisy_odom_buffer_.back().header.stamp);
    int64_t target_ns = latest_stamp.nanoseconds() - delay_ns_;

    // Find two messages that bracket the target time
    int idx_before = -1;
    int idx_after = -1;

    for (size_t i = 0; i < noisy_odom_buffer_.size(); ++i)
    {
        int64_t t = rclcpp::Time(noisy_odom_buffer_[i].header.stamp).nanoseconds();
        if (t <= target_ns)
        {
            idx_before = static_cast<int>(i);
        }
        else
        {
            idx_after = static_cast<int>(i);
            break;
        }
    }

    if (idx_before < 0 || idx_after < 0)
        return;

    const auto& odom_a = noisy_odom_buffer_[idx_before];
    const auto& odom_b = noisy_odom_buffer_[idx_after];

    int64_t t_a = rclcpp::Time(odom_a.header.stamp).nanoseconds();
    int64_t t_b = rclcpp::Time(odom_b.header.stamp).nanoseconds();

    double alpha = 0.0;
    if (t_b != t_a)
    {
        alpha = static_cast<double>(target_ns - t_a)
              / static_cast<double>(t_b - t_a);
    }

    Odometry delayed_odom = interpolate(odom_a, odom_b, alpha);
    delayed_odom.header.stamp = rclcpp::Time(target_ns, RCL_ROS_TIME);

    noisy_odom_pub_->publish(delayed_odom);

    PoseStamped delayed_pose;
    delayed_pose.header = delayed_odom.header;
    delayed_pose.pose = delayed_odom.pose.pose;
    noisy_pose_pub_->publish(delayed_pose);
}

Odometry RosOdomNoiseGenerator::interpolate(
    const Odometry& odom_a,
    const Odometry& odom_b,
    double alpha)
{
    Odometry result = odom_a;

    // Position: linear interpolation
    result.pose.pose.position.x =
        (1.0 - alpha) * odom_a.pose.pose.position.x
        + alpha * odom_b.pose.pose.position.x;
    result.pose.pose.position.y =
        (1.0 - alpha) * odom_a.pose.pose.position.y
        + alpha * odom_b.pose.pose.position.y;
    result.pose.pose.position.z =
        (1.0 - alpha) * odom_a.pose.pose.position.z
        + alpha * odom_b.pose.pose.position.z;

    // Orientation: SLERP
    Eigen::Quaterniond q_a(
        odom_a.pose.pose.orientation.w,
        odom_a.pose.pose.orientation.x,
        odom_a.pose.pose.orientation.y,
        odom_a.pose.pose.orientation.z);
    Eigen::Quaterniond q_b(
        odom_b.pose.pose.orientation.w,
        odom_b.pose.pose.orientation.x,
        odom_b.pose.pose.orientation.y,
        odom_b.pose.pose.orientation.z);

    Eigen::Quaterniond q_interp = q_a.slerp(alpha, q_b);

    result.pose.pose.orientation.w = q_interp.w();
    result.pose.pose.orientation.x = q_interp.x();
    result.pose.pose.orientation.y = q_interp.y();
    result.pose.pose.orientation.z = q_interp.z();

    // Linear velocity: linear interpolation
    result.twist.twist.linear.x =
        (1.0 - alpha) * odom_a.twist.twist.linear.x
        + alpha * odom_b.twist.twist.linear.x;
    result.twist.twist.linear.y =
        (1.0 - alpha) * odom_a.twist.twist.linear.y
        + alpha * odom_b.twist.twist.linear.y;
    result.twist.twist.linear.z =
        (1.0 - alpha) * odom_a.twist.twist.linear.z
        + alpha * odom_b.twist.twist.linear.z;

    // Angular velocity: linear interpolation
    result.twist.twist.angular.x =
        (1.0 - alpha) * odom_a.twist.twist.angular.x
        + alpha * odom_b.twist.twist.angular.x;
    result.twist.twist.angular.y =
        (1.0 - alpha) * odom_a.twist.twist.angular.y
        + alpha * odom_b.twist.twist.angular.y;
    result.twist.twist.angular.z =
        (1.0 - alpha) * odom_a.twist.twist.angular.z
        + alpha * odom_b.twist.twist.angular.z;

    return result;
}

void RosOdomNoiseGenerator::configure()
{
    // Configuration logic for noise parameters can be added here
}
