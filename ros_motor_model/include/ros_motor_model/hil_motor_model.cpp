#include "hil_motor_model.hpp"
#include "utils/converter_def.h"

HilMotorModelNode::HilMotorModelNode()
: Node("hil_motor_model_node")
{

    std::string actual_rpm_topic_name;
    std::string actual_rps_topic_name;

    actual_rpm_topic_name = "/uav/actual_rpm";
    actual_rps_topic_name = "/S550/command/motor_speed";

    // Declare and get UAV type parameter
    this->declare_parameter<std::string>("uav_type", "HEXA");
    std::string uav_type_str = this->get_parameter("uav_type").as_string();
    
    RCLCPP_INFO(this->get_logger(),
    "UAV type parameter: %s",
    uav_type_str.c_str());

    int num_motors = 6;

    if (uav_type_str == "QUAD")
    {
        sub_actual_rpm_quad_ = this->create_subscription<QuadActualRpm>(
        actual_rpm_topic_name, rclcpp::SensorDataQoS(),
        std::bind(&HilMotorModelNode::callback_actual_rpm_quad, this, 
        std::placeholders::_1));
            
        num_motors = 4;
    }
    else if (uav_type_str == "HEXA")
    {
        sub_actual_rpm_hexa_ = this->create_subscription<HexaActualRpm>
        (actual_rpm_topic_name, 
        rclcpp::SensorDataQoS(),
        std::bind(&HilMotorModelNode::callback_actual_rpm_hexa, this, 
        std::placeholders::_1));

        num_motors = 6;
    }

    pub_motor_velocity_ = this->create_publisher<Actuators>(
    actual_rps_topic_name, 1);

    timer_ = this->create_wall_timer(10ms, 
        std::bind(&HilMotorModelNode::publish_motor_velocity, this));

    actuator_msg_.velocity.reserve(num_motors);
    actuator_msg_.velocity.assign(num_motors, 0.0);

}

HilMotorModelNode::~HilMotorModelNode()
{
}

void HilMotorModelNode::callback_actual_rpm_quad(
    const QuadActualRpm::SharedPtr msg)
{
    for(size_t i = 0; i < 4; ++i)
    {
        actuator_msg_.velocity.at(i) = double(msg->rpm[i]*RPM_TO_RPS);
    }
}

void HilMotorModelNode::callback_actual_rpm_hexa(
    const HexaActualRpm::SharedPtr msg)
{
    for(size_t i = 0; i < 6; ++i)
    {
        actuator_msg_.velocity.at(i) = double(msg->rpm[i]*RPM_TO_RPS);
    }
}

void HilMotorModelNode::publish_motor_velocity()
{
    pub_motor_velocity_->publish(actuator_msg_);
}