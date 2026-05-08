cmake_minimum_required(VERSION 3.8)
project(drone_gazebo)

find_package(ament_cmake REQUIRED)
find_package(drone_description REQUIRED)

# --- Angular damping Gazebo system plugin ---
find_package(gz-sim6 QUIET)
if(NOT gz-sim6_FOUND)
  find_package(ignition-gazebo6 QUIET)
  if(ignition-gazebo6_FOUND)
    set(GZ_SIM_TARGET ignition-gazebo6::ignition-gazebo6)
  endif()
else()
  set(GZ_SIM_TARGET gz-sim6::gz-sim6)
endif()

if(DEFINED GZ_SIM_TARGET)
  add_library(AngularDamping SHARED
    src/angular_damping_plugin.cpp)
  target_link_libraries(AngularDamping ${GZ_SIM_TARGET})
  install(TARGETS AngularDamping
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "gz-sim6 / ignition-gazebo6 not found – "
                  "AngularDamping plugin will NOT be built.")
endif()

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
