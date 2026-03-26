#ifndef ROS_ODOM_DELAY_NODE_HPP
#define ROS_ODOM_DELAY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseStamped;

class RosOdomDelayNode
{
public:

    RosOdomDelayNode();
    ~RosOdomDelayNode();

    rclcpp::Node::SharedPtr getNode() { return node_; }

private:

    void odomCallback(const Odometry::SharedPtr msg);

    void timerCallback();

    /**
     * @brief Interpolate between two odometry messages
     * @param odom_a Earlier message
     * @param odom_b Later message
     * @param alpha Interpolation factor [0,1] where 0 = odom_a, 1 = odom_b
     * @return Interpolated odometry message
     */
    Odometry interpolate(const Odometry& odom_a,
                         const Odometry& odom_b,
                         double alpha);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr delayed_odom_pub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr delayed_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Circular buffer for recent odometry messages
    std::deque<Odometry> odom_buffer_;

    // Delay in nanoseconds (25 ms)
    int64_t delay_ns_;

    // Maximum buffer duration in nanoseconds
    static constexpr int64_t MAX_BUFFER_DURATION_NS = 500000000LL; // 500 ms
};

#endif // ROS_ODOM_DELAY_NODE_HPP
