cmake_minimum_required(VERSION 3.8)
project(autoware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roscco_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(adma_ros_driver_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(include)

add_executable(
  autoware_interface_node
  src/autoware_interface.cpp
)

ament_target_dependencies(
  autoware_interface_node
  rclcpp
  roscco_msgs
  std_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_control_msgs
  can_msgs
  autoware_planning_msgs
  autoware_system_msgs
  adma_ros_driver_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS autoware_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
