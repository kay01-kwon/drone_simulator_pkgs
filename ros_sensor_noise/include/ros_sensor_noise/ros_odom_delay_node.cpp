#include "ros_sensor_noise/ros_odom_delay_node.hpp"

RosOdomDelayNode::RosOdomDelayNode()
{
    node_ = rclcpp::Node::make_shared("ros_odom_delay_node");

    // Declare parameters
    node_->declare_parameter("topics.odom_input",
                             "/mavros/local_position/odom_sim");
    node_->declare_parameter("topics.delayed_odom_output",
                             "/mavros/local_position/odom_delayed");
    node_->declare_parameter("topics.delayed_pose_output",
                             "/mocap/pose_delayed");
    node_->declare_parameter("delay.delay_ms", 25.0);

    // Load parameters
    std::string odom_input =
        node_->get_parameter("topics.odom_input").as_string();
    std::string odom_output =
        node_->get_parameter("topics.delayed_odom_output").as_string();
    std::string pose_output =
        node_->get_parameter("topics.delayed_pose_output").as_string();
    double delay_ms =
        node_->get_parameter("delay.delay_ms").as_double();

    delay_ns_ = static_cast<int64_t>(delay_ms * 1e6);

    RCLCPP_INFO(node_->get_logger(), "=== Odom Delay Node ===");
    RCLCPP_INFO(node_->get_logger(), "Input topic : %s", odom_input.c_str());
    RCLCPP_INFO(node_->get_logger(), "Odom output : %s", odom_output.c_str());
    RCLCPP_INFO(node_->get_logger(), "Pose output : %s", pose_output.c_str());
    RCLCPP_INFO(node_->get_logger(), "Delay       : %.1f ms", delay_ms);

    // Subscriber & Publishers
    odom_sub_ = node_->create_subscription<Odometry>(
        odom_input, rclcpp::SensorDataQoS(),
        std::bind(&RosOdomDelayNode::odomCallback,
                  this, std::placeholders::_1));

    delayed_odom_pub_ = node_->create_publisher<Odometry>(
        odom_output, rclcpp::SensorDataQoS());

    delayed_pose_pub_ = node_->create_publisher<PoseStamped>(
        pose_output, rclcpp::SensorDataQoS());

    // Timer at 5 ms to output interpolated delayed odom
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&RosOdomDelayNode::timerCallback, this));
}

RosOdomDelayNode::~RosOdomDelayNode()
{
}

void RosOdomDelayNode::odomCallback(const Odometry::SharedPtr msg)
{
    odom_buffer_.push_back(*msg);

    // Prune old messages beyond MAX_BUFFER_DURATION_NS
    auto latest_stamp = rclcpp::Time(msg->header.stamp);
    while (!odom_buffer_.empty())
    {
        auto oldest_stamp = rclcpp::Time(odom_buffer_.front().header.stamp);
        if ((latest_stamp - oldest_stamp).nanoseconds() > MAX_BUFFER_DURATION_NS)
        {
            odom_buffer_.pop_front();
        }
        else
        {
            break;
        }
    }
}

void RosOdomDelayNode::timerCallback()
{
    if (odom_buffer_.size() < 2)
        return;

    // Target time = latest message time - delay
    auto latest_stamp = rclcpp::Time(odom_buffer_.back().header.stamp);
    auto target_ns = latest_stamp.nanoseconds() - delay_ns_;

    // Find two messages that bracket the target time
    // odom_buffer_ is sorted by timestamp (insertion order)
    int idx_before = -1;
    int idx_after = -1;

    for (size_t i = 0; i < odom_buffer_.size(); ++i)
    {
        int64_t t = rclcpp::Time(odom_buffer_[i].header.stamp).nanoseconds();
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

    // Need both a before and after sample to interpolate
    if (idx_before < 0 || idx_after < 0)
        return;

    const auto& odom_a = odom_buffer_[idx_before];
    const auto& odom_b = odom_buffer_[idx_after];

    int64_t t_a = rclcpp::Time(odom_a.header.stamp).nanoseconds();
    int64_t t_b = rclcpp::Time(odom_b.header.stamp).nanoseconds();

    double alpha = 0.0;
    if (t_b != t_a)
    {
        alpha = static_cast<double>(target_ns - t_a)
              / static_cast<double>(t_b - t_a);
    }

    Odometry delayed_odom = interpolate(odom_a, odom_b, alpha);

    // Set timestamp to the target delayed time
    delayed_odom.header.stamp =
        rclcpp::Time(target_ns, RCL_ROS_TIME);

    delayed_odom_pub_->publish(delayed_odom);

    // Publish delayed pose
    PoseStamped delayed_pose;
    delayed_pose.header = delayed_odom.header;
    delayed_pose.pose = delayed_odom.pose.pose;
    delayed_pose_pub_->publish(delayed_pose);
}

Odometry RosOdomDelayNode::interpolate(
    const Odometry& odom_a,
    const Odometry& odom_b,
    double alpha)
{
    Odometry result = odom_a;

    // --- Position: linear interpolation ---
    result.pose.pose.position.x =
        (1.0 - alpha) * odom_a.pose.pose.position.x
        + alpha * odom_b.pose.pose.position.x;
    result.pose.pose.position.y =
        (1.0 - alpha) * odom_a.pose.pose.position.y
        + alpha * odom_b.pose.pose.position.y;
    result.pose.pose.position.z =
        (1.0 - alpha) * odom_a.pose.pose.position.z
        + alpha * odom_b.pose.pose.position.z;

    // --- Orientation: SLERP ---
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

    // --- Linear velocity: linear interpolation ---
    result.twist.twist.linear.x =
        (1.0 - alpha) * odom_a.twist.twist.linear.x
        + alpha * odom_b.twist.twist.linear.x;
    result.twist.twist.linear.y =
        (1.0 - alpha) * odom_a.twist.twist.linear.y
        + alpha * odom_b.twist.twist.linear.y;
    result.twist.twist.linear.z =
        (1.0 - alpha) * odom_a.twist.twist.linear.z
        + alpha * odom_b.twist.twist.linear.z;

    // --- Angular velocity: linear interpolation ---
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
