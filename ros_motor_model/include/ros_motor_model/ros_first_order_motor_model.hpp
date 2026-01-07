#ifndef ROS_FIRST_ORDER_MOTOR_MODEL_HPP
#define ROS_FIRST_ORDER_MOTOR_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ros2_libcanard_msgs/msg/quad_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/quad_actual_rpm.hpp>

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include "model/first_order_motor_model.hpp"
#include "utils/type_def.h"

using namespace std::chrono_literals;

using ros2_libcanard_msgs::msg::QuadCmdRaw;
using ros2_libcanard_msgs::msg::QuadActualRpm;

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

using actuator_msgs::msg::Actuators;


class RosFirstOrderMotorModelNode : public rclcpp::Node
{
    public:

    RosFirstOrderMotorModelNode();
    ~RosFirstOrderMotorModelNode();

    private:

    void callback_cmd_raw_quad(const QuadCmdRaw::SharedPtr msg);

    void callback_cmd_raw_hexa(const HexaCmdRaw::SharedPtr msg);

    void publish_motor_velocity();

    void motor_model_initialize();

    void allocate_motor_models(const size_t &num_motors,
    const FirstOrderMotorParams &params);

    rclcpp::Subscription<QuadCmdRaw>::SharedPtr sub_cmd_raw_quad_;
    rclcpp::Subscription<HexaCmdRaw>::SharedPtr sub_cmd_raw_hexa_;

    rclcpp::Publisher<QuadActualRpm>::SharedPtr pub_actual_rpm_quad_;
    rclcpp::Publisher<HexaActualRpm>::SharedPtr pub_actual_rpm_hexa_;

    rclcpp::Publisher<Actuators>::SharedPtr pub_actual_rps_;

    std::vector<FirstOrderMotorModel*> motor_models_;
    rclcpp::TimerBase::SharedPtr timer_;

    QuadCmdRaw quad_cmd_raw_msg_;
    HexaCmdRaw hexa_cmd_raw_msg_;

    QuadActualRpm quad_actual_rpm_msg_;
    HexaActualRpm hexa_actual_rpm_msg_;
    Actuators actuator_msg_;

    UAVType uav_type_;

    double time_curr_{0.0};

    bool is_time_initialized_{false};

};

#endif