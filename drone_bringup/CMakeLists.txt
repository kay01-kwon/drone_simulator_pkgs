cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(drone_description REQUIRED)
find_package(drone_gazebo REQUIRED)


# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install proejct config files
install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)


ament_package()
