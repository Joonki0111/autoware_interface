cmake_minimum_required(VERSION 3.8)
project(roscco_to_aw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(roscco_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)

add_library(
  roscco_to_aw SHARED
  src/roscco_to_aw.cpp
)

ament_target_dependencies(
  roscco_to_aw
  rclcpp
  rclcpp_components
  roscco_msgs
  std_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_control_msgs
  can_msgs
)

rclcpp_components_register_nodes(
  roscco_to_aw
  "roscco_component::RosccoToAW"
)

install(TARGETS 
  roscco_to_aw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
