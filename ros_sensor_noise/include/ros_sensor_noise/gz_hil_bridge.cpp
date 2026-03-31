#include "ros_sensor_noise/gz_hil_bridge.hpp"

GzHilBridge::GzHilBridge()
{
    node_ = rclcpp::Node::make_shared("gz_hil_bridge");

    node_->declare_parameter("topics.imu_input", "/S550/imu");
    node_->declare_parameter("topics.odom_input", "/S550/ground_truth/odom");
    node_->declare_parameter("topics.hil_sensor_output", "/mavros/hil/imu_ned");

    std::string imu_topic =
        node_->get_parameter("topics.imu_input").as_string();
    std::string odom_topic =
        node_->get_parameter("topics.odom_input").as_string();
    std::string hil_topic =
        node_->get_parameter("topics.hil_sensor_output").as_string();

    RCLCPP_INFO(node_->get_logger(), "=== GZ HIL Bridge ===");
    RCLCPP_INFO(node_->get_logger(), "IMU input:  %s", imu_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "Odom input: %s", odom_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "HIL output: %s", hil_topic.c_str());

    imu_sub_ = node_->create_subscription<Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&GzHilBridge::imuCallback, this, std::placeholders::_1));

    odom_sub_ = node_->create_subscription<Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&GzHilBridge::odomCallback, this, std::placeholders::_1));

    hil_sensor_pub_ = node_->create_publisher<HilSensor>(hil_topic, 10);
}

void GzHilBridge::odomCallback(const Odometry::SharedPtr msg)
{
    // Store altitude for barometer calculation
    latest_alt_ = msg->pose.pose.position.z;
}

void GzHilBridge::imuCallback(const Imu::SharedPtr msg)
{
    HilSensor hil;
    hil.header.stamp = node_->now();

    // --- Accelerometer: pass as FLU (MAVROS hil plugin converts to FRD) ---
    hil.acc.x = msg->linear_acceleration.x;
    hil.acc.y = msg->linear_acceleration.y;
    hil.acc.z = msg->linear_acceleration.z;

    // --- Gyroscope: pass as FLU (MAVROS hil plugin converts to FRD) ---
    hil.gyro.x = msg->angular_velocity.x;
    hil.gyro.y = msg->angular_velocity.y;
    hil.gyro.z = msg->angular_velocity.z;

    // --- Magnetometer (ENU frame, Gauss) - MAVROS converts to NED/FRD ---
    hil.mag.x = mag_east_;
    hil.mag.y = mag_north_;
    hil.mag.z = -mag_down_;

    // --- Barometer from altitude ---
    // ISA: P = 1013.25 * (1 - 2.25577e-5 * h)^5.25588
    double h = latest_alt_;
    hil.abs_pressure = 1013.25 * std::pow(1.0 - 2.25577e-5 * h, 5.25588);
    hil.diff_pressure = 0.0;
    hil.pressure_alt = static_cast<float>(h);
    hil.temperature = 25.0;

    // All fields updated
    // bits: acc(0-2) gyro(3-5) mag(6-8) pressure(9) diff(10) alt(11) temp(12)
    hil.fields_updated = 0x1FFF;

    hil_sensor_pub_->publish(hil);
}
