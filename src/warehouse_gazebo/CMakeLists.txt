cmake_minimum_required(VERSION 3.8)
project(warehouse_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)

# Generate ArUco marker textures/material scripts for spawned parcels before
# the models/ directory below gets installed. Idempotent and non-fatal if
# OpenCV isn't available at configure time (see the script's own docstring).
find_program(PYTHON3_EXECUTABLE NAMES python3 python)
if(PYTHON3_EXECUTABLE)
  execute_process(
    COMMAND ${PYTHON3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_parcel_marker_assets.py
    RESULT_VARIABLE _parcel_marker_gen_result
  )
endif()

# ── Conveyor belt velocity plugin ──────────────────────────────────────────
add_library(conveyor_belt_plugin SHARED
  src/conveyor_belt_plugin.cpp
)
ament_target_dependencies(conveyor_belt_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)
install(
  TARGETS conveyor_belt_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install config (parcel_catalog.yaml)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install models (includes parcel_markers/ generated above, if present)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install product spawner script
install(
  PROGRAMS warehouse_simulation/product_spawner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME product_spawner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()