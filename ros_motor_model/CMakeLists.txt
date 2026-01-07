cmake_minimum_required(VERSION 3.8)
project(ros_motor_model)

set(CMAKE_CXX_STANDARD 17)

set(motor_model_src
  include/model/first_order_motor_model.cpp
  include/model/second_order_motor_model.cpp
  include/ros_motor_model/ros_first_order_motor_model.cpp
  include/ros_motor_model/ros_second_order_motor_model.cpp
  include/ros_motor_model/hil_motor_model.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_libcanard_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

include_directories(
  include
)

add_library(motor_model_lib
  ${motor_model_src}
)

ament_target_dependencies(
  motor_model_lib
  rclcpp
  rcl_interfaces
  std_msgs
  ros2_libcanard_msgs
  actuator_msgs
)

# First Order Motor Model Node
add_executable(
  ros_first_order_motor_model
  src/ros_first_order_motor_model_node.cpp
)

ament_target_dependencies(
  ros_first_order_motor_model
  rclcpp
  rcl_interfaces
  std_msgs
  ros2_libcanard_msgs
  actuator_msgs
)

target_link_libraries(
  ros_first_order_motor_model
  motor_model_lib
)

# Second Order Motor Model Node
add_executable(
  ros_second_order_motor_model
  src/ros_second_order_motor_model_node.cpp
)

ament_target_dependencies(
  ros_second_order_motor_model
  rclcpp
  rcl_interfaces
  std_msgs
  ros2_libcanard_msgs
  actuator_msgs
)

target_link_libraries(
  ros_second_order_motor_model
  motor_model_lib
)

# HIL Motor Model Node
add_executable(
  hil_motor_model
  src/hil_motor_model_node.cpp
)

ament_target_dependencies(
  hil_motor_model
  rclcpp
  rcl_interfaces
  std_msgs
  ros2_libcanard_msgs
  actuator_msgs
)

target_link_libraries(
  hil_motor_model
  motor_model_lib
)

install(
  TARGETS 
    ros_first_order_motor_model 
    ros_second_order_motor_model
    hil_motor_model
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()