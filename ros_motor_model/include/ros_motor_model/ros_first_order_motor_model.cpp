#include "ros_first_order_motor_model.hpp"
#include "utils/converter_def.h"

RosFirstOrderMotorModelNode::RosFirstOrderMotorModelNode() 
: Node("ros_first_order_motor_model_node")
{
    std::string cmd_raw_topic_name;
    std::string actual_rpm_topic_name;
    std::string actual_rps_topic_name;

    cmd_raw_topic_name = "/uav/cmd_raw";
    actual_rpm_topic_name = "/uav/actual_rpm";
    actual_rps_topic_name = "/S550/command/motor_speed";

    RCLCPP_INFO(this->get_logger(), 
    "Initializing ROS Motor Model Node...");
    // Initialization code here

    // Declare and get UAV type parameter
    this->declare_parameter<std::string>("motor.uav_type", "HEXA");
    std::string uav_type_str = this->get_parameter("motor.uav_type").as_string();
    
    RCLCPP_INFO(this->get_logger(),
    "UAV type parameter: %s",
    uav_type_str.c_str());

    FirstOrderMotorParams motor_params;

    // Declare and get motor time constant parameters
    this->declare_parameter<double>("motor.time_const_up", 0.01);
    this->declare_parameter<double>("motor.time_const_down", 0.01);
    motor_params.timeConstUp = this->get_parameter("motor.time_const_up").as_double();
    motor_params.timeConstDown = this->get_parameter("motor.time_const_down").as_double();
    
    RCLCPP_INFO(this->get_logger(),
    "Motor time constants - Up: %.4f, Down: %.4f",
    motor_params.timeConstUp, motor_params.timeConstDown);

    size_t num_motors = 6;

    pub_actual_rpm_quad_ = nullptr;
    pub_actual_rpm_hexa_ = nullptr;


    if (uav_type_str == "QUAD")
    {
        uav_type_ = UAVType::QUAD;
        RCLCPP_INFO(this->get_logger(),
        "UAV type set to QUAD");

        sub_cmd_raw_quad_ = this->create_subscription<QuadCmdRaw>(
        cmd_raw_topic_name, 5,
        std::bind(&RosFirstOrderMotorModelNode::callback_cmd_raw_quad, 
        this, std::placeholders::_1)
        );

        pub_actual_rpm_quad_ = this->create_publisher<QuadActualRpm>(
        actual_rpm_topic_name, rclcpp::SensorDataQoS());

        allocate_motor_models(num_motors, motor_params);

    }
    else if (uav_type_str == "HEXA")
    {
        uav_type_ = UAVType::HEXA;
        RCLCPP_INFO(this->get_logger(),
        "UAV type set to HEXA");

        sub_cmd_raw_hexa_ = this->create_subscription<HexaCmdRaw>(
        cmd_raw_topic_name, 5,
        std::bind(&RosFirstOrderMotorModelNode::callback_cmd_raw_hexa,
        this, std::placeholders::_1)
        );

        pub_actual_rpm_hexa_ = this->create_publisher<HexaActualRpm>(
        actual_rpm_topic_name, rclcpp::SensorDataQoS());

        allocate_motor_models(num_motors, motor_params);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
        "Invalid UAV type parameter. Supported types are QUAD and HEXA.");
        throw std::runtime_error("Invalid UAV type parameter");
    }

    pub_actual_rps_ = this->create_publisher<Actuators>(
    actual_rps_topic_name, 1);

    timer_ = this->create_wall_timer(
    10ms, std::bind(&RosFirstOrderMotorModelNode::publish_motor_velocity, this
    ));

    time_curr_ = this->now().seconds();

    actuator_msg_.velocity.reserve(num_motors);
    actuator_msg_.velocity.assign(num_motors, 0.0);

}

RosFirstOrderMotorModelNode::~RosFirstOrderMotorModelNode()
{
    // Cleanup code here
    for (auto motor_model : motor_models_)
    {
        delete motor_model;
    }
    motor_models_.clear();
}

void RosFirstOrderMotorModelNode::callback_cmd_raw_quad(const QuadCmdRaw::SharedPtr msg)
{

    quad_cmd_raw_msg_ = *msg;
}

void RosFirstOrderMotorModelNode::callback_cmd_raw_hexa(const HexaCmdRaw::SharedPtr msg)
{
    hexa_cmd_raw_msg_ = *msg;
}

void RosFirstOrderMotorModelNode::publish_motor_velocity()
{
    double cmd = 0.0;
    double actual_vel = 0.0;

    time_curr_ = this->now().seconds();

    actuator_msg_.header.stamp = this->now();
    
    if( !is_time_initialized_ )
    {
        motor_model_initialize();
        is_time_initialized_ = true;
        return;
    }

    // Publish motor velocity implementation
    if (uav_type_ == UAVType::QUAD)
    {
        quad_actual_rpm_msg_.header.stamp = this->now();

        for( size_t i = 0; i < motor_models_.size(); ++i)
        {
            cmd = (double) quad_cmd_raw_msg_.cmd_raw[i] * MAX_RPM/MAX_BIT; // Convert command to RPM
            cmd = cmd * RPM_TO_RPS; // Convert RPM to RPS
            
            motor_models_.at(i)->set_state(cmd, time_curr_);
            motor_models_.at(i)->get_state(actual_vel);
            actuator_msg_.velocity.at(i) = actual_vel;

            actual_vel = actual_vel * RPS_TO_RPM; // Convert RPS to RPM
            quad_actual_rpm_msg_.rpm[i] = static_cast<int16_t>(actual_vel);
        }

        pub_actual_rpm_quad_->publish(quad_actual_rpm_msg_);
    }
    else if (uav_type_ == UAVType::HEXA)
    {
        hexa_actual_rpm_msg_.header.stamp = this->now();

        for( size_t i = 0; i < motor_models_.size(); ++i)
        {
            cmd = (double) hexa_cmd_raw_msg_.cmd_raw[i] * MAX_RPM/MAX_BIT; // Convert command to RPM
            cmd = cmd * RPM_TO_RPS; // Convert RPM to RPS

            motor_models_.at(i)->set_state(cmd, time_curr_);            
            motor_models_.at(i)->get_state(actual_vel);
            actuator_msg_.velocity.at(i) = actual_vel;

            // printf("Actuator motor %zu velocity (RPS): %.4f\t", i, actual_vel);
            
            actual_vel = actual_vel * RPS_TO_RPM; // Convert RPS to RPM
            hexa_actual_rpm_msg_.rpm[i] = static_cast<int16_t>(actual_vel);
        }
        // printf("\n");
        pub_actual_rpm_hexa_->publish(hexa_actual_rpm_msg_);
    }

    pub_actual_rps_->publish(actuator_msg_);
}

void RosFirstOrderMotorModelNode::allocate_motor_models(const size_t &num_motors,
const FirstOrderMotorParams &params)
{
    // Allocate motor models implementation
    motor_models_.clear();
    motor_models_.reserve(num_motors);
    for ( size_t i = 0; i < num_motors; ++i)
    {
        motor_models_.emplace_back(new FirstOrderMotorModel(params));
    }

    printf("Allocated %zu motor models.\n", motor_models_.size());
}

void RosFirstOrderMotorModelNode::motor_model_initialize()
{
    printf("Initializing motor models at time: %.4f\n", time_curr_);
    for( size_t i = 0; i < motor_models_.size(); ++i)
    {
        printf("Initializing motor model %zu at time: %.4f\n", i, time_curr_);
        motor_models_.at(i)->set_initial_time(time_curr_);
    }
}