#include "ros_second_order_motor_model.hpp"
#include "utils/converter_def.h"

RosSecondOrderMotorModelNode::RosSecondOrderMotorModelNode() : Node("ros_second_order_motor_model_node")
{

    std::string cmd_raw_topic_name = "/uav/cmd_raw";;
    std::string actual_rpm_topic_name = "/uav/actual_rpm";
    std::string actual_rps_topic_name = "/S550/command/motor_speed";

    RCLCPP_INFO(this->get_logger(),
                "Initializing ROS Second Order Motor Model Node...");

    
    // Declare and get motor parameters
    SecondOrderMotorParams motor_params;

    // Declare and get UAV type parameter
    this->declare_parameter<std::string>("motor.uav_type", "HEXA");
    std::string uav_type_str = this->get_parameter("motor.uav_type").as_string();

    this->declare_parameter<double>("motor.alpha_max", motor_params.alpha_max);
    this->declare_parameter<double>("motor.p1", motor_params.p1);
    this->declare_parameter<double>("motor.p2", motor_params.p2);
    this->declare_parameter<double>("motor.p3", motor_params.p3);
    this->declare_parameter<double>("motor.jerk_max", motor_params.jerk_max);

    motor_params.alpha_max = this->get_parameter("motor.alpha_max").as_double();
    motor_params.p1 = this->get_parameter("motor.p1").as_double();
    motor_params.p2 = this->get_parameter("motor.p2").as_double();
    motor_params.p3 = this->get_parameter("motor.p3").as_double();
    motor_params.jerk_max = this->get_parameter("motor.jerk_max").as_double();

    // Declare and get noise parameters
    this->declare_parameter<bool>("motor.noise.enabled", false);
    this->declare_parameter<double>("motor.noise.mean", 0.0);
    this->declare_parameter<double>("motor.noise.std_dev", 0.0);

    noise_enabled_ = this->get_parameter("motor.noise.enabled").as_bool();
    noise_mean_ = this->get_parameter("motor.noise.mean").as_double();
    noise_std_dev_ = this->get_parameter("motor.noise.std_dev").as_double();

    // Initialize random number generator
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
    noise_distribution_ = std::normal_distribution<double>(noise_mean_, noise_std_dev_);

    RCLCPP_INFO(this->get_logger(),
                "Noise Parameters: enabled=%s, mean=%.2f, std_dev=%.2f",
                noise_enabled_ ? "true" : "false",
                noise_mean_,
                noise_std_dev_);

    size_t num_motors = 6;

    if (uav_type_str == "QUAD")
    {
        uav_type_ = UAVType::QUAD;
        RCLCPP_INFO(this->get_logger(), "Configuring for QUAD UAV");

        num_motors = 4;

        allocate_motor_models(num_motors, motor_params);

        sub_cmd_raw_quad_ = this->create_subscription<QuadCmdRaw>(
            cmd_raw_topic_name, 5,
            std::bind(&RosSecondOrderMotorModelNode::callback_cmd_raw_quad, this, std::placeholders::_1));
        
        pub_actual_rpm_quad_ = this->create_publisher<QuadActualRpm>(
            actual_rpm_topic_name, rclcpp::SensorDataQoS());

    }
    else if (uav_type_str == "HEXA")
    {
        uav_type_ = UAVType::HEXA;
        RCLCPP_INFO(this->get_logger(), "Configuring for HEXA UAV");
        
        num_motors = 6;

        allocate_motor_models(num_motors, motor_params);
        
        sub_cmd_raw_hexa_ = this->create_subscription<HexaCmdRaw>(
            cmd_raw_topic_name, 5,
            std::bind(&RosSecondOrderMotorModelNode::callback_cmd_raw_hexa, this, std::placeholders::_1));
        
        pub_actual_rpm_hexa_ = this->create_publisher<HexaActualRpm>(
            actual_rpm_topic_name, rclcpp::SensorDataQoS());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported UAV type: %s", uav_type_str.c_str());
        throw std::runtime_error("Unsupported UAV type");
    }

    RCLCPP_INFO(this->get_logger(),
                "Motor Parameters: alpha_max=%.2f, p1=%.6f, p2=%.10f, p3=%.6f, jerk_max=%.2f",
                motor_params.alpha_max,
                motor_params.p1,
                motor_params.p2,
                motor_params.p3,
                motor_params.jerk_max);

    pub_actual_rps_ = this->create_publisher<Actuators>(
        actual_rps_topic_name, 5);
    
    timer_ = this->create_wall_timer(
    10ms, std::bind(&RosSecondOrderMotorModelNode::publish_motor_velocity, this));


    time_curr_ = this->now().seconds();

    actuators_msg_.velocity.reserve(num_motors);
    actuators_msg_.velocity.assign(num_motors, 0.0);

}

RosSecondOrderMotorModelNode::~RosSecondOrderMotorModelNode()
{
    for (auto motor_model : motor_models_)
    {
        delete motor_model;
    }
}

void RosSecondOrderMotorModelNode::callback_cmd_raw_quad(const QuadCmdRaw::SharedPtr msg)
{
    quad_cmd_raw_msg_ = *msg;
}

void RosSecondOrderMotorModelNode::callback_cmd_raw_hexa(const HexaCmdRaw::SharedPtr msg)
{
    hexa_cmd_raw_msg_ = *msg;
}

void RosSecondOrderMotorModelNode::publish_motor_velocity()
{

    if(!is_time_initialized_)
    {
        motor_model_initialize();
        return;
    }

    time_curr_ = this->now().seconds();

    actuators_msg_.header.stamp = this->now();

    switch(uav_type_)
    {
        case UAVType::QUAD:
            quad_actual_rpm_msg_.header.stamp = this->now();

            for(size_t i = 0; i < motor_models_.size(); ++i)
            {
                // Convert cmd_raw(Bit) to cmd_rpm - Maxon motor firmware specification
                double cmd_rpm = quad_cmd_raw_msg_.cmd_raw[i] * MAX_RPM/MAX_BIT;

                double actual_rpm;

                motor_models_.at(i)->set_state(cmd_rpm, time_curr_);
                motor_models_.at(i)->get_state(actual_rpm);

                // Add noise if enabled
                if (noise_enabled_)
                {
                    double noise = noise_distribution_(random_generator_);
                    actual_rpm += noise;
                }

                // Gazebo ROS interface expects rad/s
                actuators_msg_.velocity.at(i) = actual_rpm * RPM_TO_RPS;

                // Fill in the actual RPM messages according to UAV type
                quad_actual_rpm_msg_.rpm[i] = static_cast<int32_t>(actual_rpm);
                quad_actual_rpm_msg_.acceleration[i] = 0;
            }
            pub_actual_rpm_quad_->publish(quad_actual_rpm_msg_);

            break;
        case UAVType::HEXA:
            hexa_actual_rpm_msg_.header.stamp = this->now();

            for(size_t i = 0; i < motor_models_.size(); ++i)
            {
                // Convert cmd_raw(Bit) to cmd_rpm - Maxon motor firmware specification
                double cmd_rpm = hexa_cmd_raw_msg_.cmd_raw[i] * MAX_RPM/MAX_BIT;

                double actual_rpm;

                motor_models_.at(i)->set_state(cmd_rpm, time_curr_);
                motor_models_.at(i)->get_state(actual_rpm);

                // Add noise if enabled
                if (noise_enabled_)
                {
                    double noise = noise_distribution_(random_generator_);
                    actual_rpm += noise;
                }

                // Gazebo ROS interface expects rad/s
                actuators_msg_.velocity.at(i) = actual_rpm * RPM_TO_RPS;

                // Fill in the actual RPM messages according to UAV type
                hexa_actual_rpm_msg_.rpm[i] = static_cast<int32_t>(actual_rpm);
                hexa_actual_rpm_msg_.acceleration[i] = 0;
            }
            pub_actual_rpm_hexa_->publish(hexa_actual_rpm_msg_);

            break;
        default:
            RCLCPP_ERROR(this->get_logger(), 
            "Unsupported UAV type during publish_motor_velocity");
            return;
    }

    // Publish the Actuators message to ROS Gazebo interface
    pub_actual_rps_->publish(actuators_msg_);

}

void RosSecondOrderMotorModelNode::motor_model_initialize()
{
    RCLCPP_INFO(this->get_logger(),
                "Initializing motor models at time: %.6f",
                time_curr_);
    for(size_t i = 0; i < motor_models_.size(); ++i)
    {
        motor_models_[i]->set_initial_time(time_curr_);
    }
    is_time_initialized_ = true;
}

void RosSecondOrderMotorModelNode::allocate_motor_models(const size_t &num_motors,
    const SecondOrderMotorParams &params)
{
    motor_models_.clear();
    motor_models_.reserve(num_motors);
    for (size_t i = 0; i < num_motors; ++i)
    {
        motor_models_.emplace_back(new SecondOrderMotorModel(params));
    }
    RCLCPP_INFO(this->get_logger(),
                "Allocated %zu motor models.", num_motors);
}