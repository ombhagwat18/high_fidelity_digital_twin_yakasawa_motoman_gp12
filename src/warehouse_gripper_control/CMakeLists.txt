cmake_minimum_required(VERSION 3.8)
project(warehouse_gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(vacuum_gripper_plugin SHARED
  src/vacuum_gripper_plugin.cpp
)

ament_target_dependencies(vacuum_gripper_plugin
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_msgs"
  "rclcpp"
  "std_srvs"
  "std_msgs"
  "diagnostic_msgs"
)

install(
  TARGETS vacuum_gripper_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
