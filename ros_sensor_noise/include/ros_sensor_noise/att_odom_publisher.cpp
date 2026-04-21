#include "ros_sensor_noise/att_odom_publisher.hpp"

AttOdomPublisher::AttOdomPublisher()
    : Node("att_odom_publisher")
{
    this->declare_parameter("model_origin_z", 0.5);
    this->declare_parameter("pivot_offset_z", 0.02);
    this->declare_parameter("imu_offset_z", 0.052);
    this->declare_parameter("odom_frame", std::string("S550/odom"));
    this->declare_parameter("child_frame", std::string("imu_link"));
    this->declare_parameter("odom_topic", std::string("/S550/ground_truth/odom"));

    model_origin_z_ = this->get_parameter("model_origin_z").as_double();
    pivot_offset_z_ = this->get_parameter("pivot_offset_z").as_double();
    imu_offset_z_ = this->get_parameter("imu_offset_z").as_double();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    auto odom_topic = this->get_parameter("odom_topic").as_string();

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&AttOdomPublisher::jointStateCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    RCLCPP_INFO(this->get_logger(),
        "AttOdomPublisher: model_z=%.3f pivot_offset=%.3f imu_offset=%.3f",
        model_origin_z_, pivot_offset_z_, imu_offset_z_);
}

void AttOdomPublisher::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::unordered_map<std::string, double> joint_pos;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (i < msg->position.size()) {
            joint_pos[msg->name[i]] = msg->position[i];
        }
    }

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (joint_pos.count("roll_joint")) roll = joint_pos["roll_joint"];
    if (joint_pos.count("pitch_joint")) pitch = joint_pos["pitch_joint"];
    if (joint_pos.count("yaw_joint")) yaw = joint_pos["yaw_joint"];

    // FK: world -> anchor (0,0,model_origin_z) -> roll(X) -> pitch(Y) -> yaw(Z)
    //     -> base_link (0,0,pivot_offset_z above pivot) -> imu_link (0,0,imu_offset_z above base)
    Eigen::Quaterniond q_roll(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond q_world = q_roll * q_pitch * q_yaw;
    q_world.normalize();

    // Position: pivot is at (0, 0, model_origin_z) in world.
    // base_link is pivot_offset_z above pivot in body frame.
    // imu_link is imu_offset_z above base_link in body frame.
    double total_offset = pivot_offset_z_ + imu_offset_z_;
    Eigen::Vector3d offset_body(0.0, 0.0, total_offset);
    Eigen::Vector3d offset_world = q_world * offset_body;

    Eigen::Vector3d position(0.0, 0.0, model_origin_z_);
    position += offset_world;

    // Angular velocity from joint velocities
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
    if (joint_pos.count("roll_joint") && msg->velocity.size() > 0) {
        std::unordered_map<std::string, double> joint_vel;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->velocity.size()) {
                joint_vel[msg->name[i]] = msg->velocity[i];
            }
        }
        double roll_vel = joint_vel.count("roll_joint") ? joint_vel["roll_joint"] : 0.0;
        double pitch_vel = joint_vel.count("pitch_joint") ? joint_vel["pitch_joint"] : 0.0;
        double yaw_vel = joint_vel.count("yaw_joint") ? joint_vel["yaw_joint"] : 0.0;
        // Approximate body-frame angular velocity
        angular_vel = q_world.inverse() * Eigen::Vector3d(roll_vel, pitch_vel, yaw_vel);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;

    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.x = q_world.x();
    odom.pose.pose.orientation.y = q_world.y();
    odom.pose.pose.orientation.z = q_world.z();
    odom.pose.pose.orientation.w = q_world.w();

    odom.twist.twist.angular.x = angular_vel.x();
    odom.twist.twist.angular.y = angular_vel.y();
    odom.twist.twist.angular.z = angular_vel.z();

    odom_pub_->publish(odom);
}
