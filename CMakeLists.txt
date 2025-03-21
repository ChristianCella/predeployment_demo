cmake_minimum_required(VERSION 3.8)
project(predeployment_demo)

# Use stricter compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(abb_robot_msgs REQUIRED)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  #include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${abb_robot_msgs_INCLUDE_DIRS}
)

# --------------------------------------------------
# Add C++ Executables
# --------------------------------------------------
add_executable(manipulability_analyzer src/manipulability_analyzer.cpp)
ament_target_dependencies(manipulability_analyzer
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  Eigen3
)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  Eigen3
  abb_robot_msgs
)

add_executable(only_robot src/only_robot.cpp)
ament_target_dependencies(only_robot
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  Eigen3
  abb_robot_msgs
)

# --------------------------------------------------
# Install C++ targets
# --------------------------------------------------
install(TARGETS
  manipulability_analyzer
  pick_and_place
  only_robot
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Install Python scripts
# --------------------------------------------------
install(PROGRAMS
  scripts/move_robot_cartesian.py
  scripts/move_robot_joints.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Export include directory (if you add headers later)
# --------------------------------------------------
#install(DIRECTORY include/
#  DESTINATION include/
#)

# --------------------------------------------------
# Optional: Enable testing and linters
# --------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
