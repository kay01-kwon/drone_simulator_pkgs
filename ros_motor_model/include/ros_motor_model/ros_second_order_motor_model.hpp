#ifndef ROS_SECOND_ORDER_MOTOR_MODEL_HPP
#define ROS_SECOND_ORDER_MOTOR_MODEL_HPP


#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include <ros2_libcanard_msgs/msg/quad_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/quad_actual_rpm.hpp>

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include "model/second_order_motor_model.hpp"

using namespace std::chrono_literals;

using ros2_libcanard_msgs::msg::QuadCmdRaw;
using ros2_libcanard_msgs::msg::QuadActualRpm;

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

using actuator_msgs::msg::Actuators;

class RosSecondOrderMotorModelNode : public rclcpp::Node
{
    public:

    RosSecondOrderMotorModelNode();
    ~RosSecondOrderMotorModelNode();

    private:

    void callback_cmd_raw_quad(const QuadCmdRaw::SharedPtr msg);

    void callback_cmd_raw_hexa(const HexaCmdRaw::SharedPtr msg);

    void publish_motor_velocity();

    void motor_model_initialize();

    void allocate_motor_models(const size_t &num_motors,
    const SecondOrderMotorParams &params);

    rclcpp::Subscription<QuadCmdRaw>::SharedPtr sub_cmd_raw_quad_{nullptr};
    rclcpp::Subscription<HexaCmdRaw>::SharedPtr sub_cmd_raw_hexa_{nullptr};

    rclcpp::Publisher<QuadActualRpm>::SharedPtr pub_actual_rpm_quad_{nullptr};
    rclcpp::Publisher<HexaActualRpm>::SharedPtr pub_actual_rpm_hexa_{nullptr};
    rclcpp::Publisher<Actuators>::SharedPtr pub_actual_rps_{nullptr};

    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    std::vector<SecondOrderMotorModel*> motor_models_;

    UAVType uav_type_;

    QuadCmdRaw quad_cmd_raw_msg_;
    HexaCmdRaw hexa_cmd_raw_msg_;

    QuadActualRpm quad_actual_rpm_msg_;
    HexaActualRpm hexa_actual_rpm_msg_;
    Actuators actuators_msg_;

    double time_curr_{0.0};

    bool is_time_initialized_{false};

    // Noise parameters
    bool noise_enabled_{false};
    double noise_mean_{0.0};
    double noise_std_dev_{0.0};
    std::default_random_engine random_generator_;
    std::normal_distribution<double> noise_distribution_;
};



#endif // ROS_SECOND_ORDER_MOTOR_MODEL_HPP