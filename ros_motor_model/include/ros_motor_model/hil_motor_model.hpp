#ifndef HIL_MOTOR_MODEL_HPP
#define HIL_MOTOR_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ros2_libcanard_msgs/msg/quad_actual_rpm.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>

#include <actuator_msgs/msg/actuators.hpp>

#include "utils/type_def.h"

using namespace std::chrono_literals;

using ros2_libcanard_msgs::msg::QuadActualRpm;
using ros2_libcanard_msgs::msg::HexaActualRpm;

using actuator_msgs::msg::Actuators;


class HilMotorModelNode : public rclcpp::Node
{
    public:

    HilMotorModelNode();

    ~HilMotorModelNode();

    private:

    void callback_actual_rpm_quad(const QuadActualRpm::SharedPtr msg);

    void callback_actual_rpm_hexa(const HexaActualRpm::SharedPtr msg);

    void publish_motor_velocity();

    rclcpp::Subscription<QuadActualRpm>::SharedPtr sub_actual_rpm_quad_{nullptr};
    rclcpp::Subscription<HexaActualRpm>::SharedPtr sub_actual_rpm_hexa_{nullptr};

    rclcpp::Publisher<Actuators>::SharedPtr pub_motor_velocity_{nullptr};

    rclcpp::TimerBase::SharedPtr timer_;

    Actuators actuator_msg_;

};


#endif // HIL_MOTOR_MODEL_HPP