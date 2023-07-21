################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(swerve_cartographer)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${Boost_INCLUDE_DIRS}
)

set(dependencies
  "rclcpp"
  "tf2_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
add_executable(pose_publisher
  src/main_pub_pose.cpp
  src/pubPose.cpp
)
target_link_libraries(pose_publisher
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(pose_publisher
  ${dependencies}
)

# ###############################################################################
# Install
# ###############################################################################
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

# ###############################################################################
# Macro for ament package
# ###############################################################################
ament_package()
