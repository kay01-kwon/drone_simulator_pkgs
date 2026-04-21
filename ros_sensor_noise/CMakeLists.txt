cmake_minimum_required(VERSION 3.8)
project(ros_sensor_noise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(ros_odom_noise_generator
  src/main.cpp
  include/ros_sensor_noise/ros_odom_noise_generator.cpp
)

# Link dependencies
ament_target_dependencies(ros_odom_noise_generator
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(ros_odom_noise_generator
  Eigen3::Eigen
)

add_executable(att_odom_publisher
  src/att_odom_main.cpp
  include/ros_sensor_noise/att_odom_publisher.cpp
)

ament_target_dependencies(att_odom_publisher
  rclcpp
  nav_msgs
  sensor_msgs
)

target_link_libraries(att_odom_publisher
  Eigen3::Eigen
)

add_executable(gz_hil_bridge
  src/hil_bridge_main.cpp
  include/ros_sensor_noise/gz_hil_bridge.cpp
)

ament_target_dependencies(gz_hil_bridge
  rclcpp
  sensor_msgs
  nav_msgs
  mavros_msgs
)

# Install executable
install(TARGETS
  ros_odom_noise_generator
  att_odom_publisher
  gz_hil_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
