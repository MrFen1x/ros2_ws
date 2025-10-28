cmake_minimum_required(VERSION 3.8)
project(my_webots_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../libraries/youbot_control/include
  ${WEBOTS_HOME}/include/controller/c
)

add_library(youbot_control STATIC
  ${PROJECT_SOURCE_DIR}/../libraries/youbot_control/src/arm.c
  ${PROJECT_SOURCE_DIR}/../libraries/youbot_control/src/base.c
  ${PROJECT_SOURCE_DIR}/../libraries/youbot_control/src/gripper.c
  ${PROJECT_SOURCE_DIR}/../libraries/youbot_control/src/tiny_math.c
)

add_executable(my_robot_node src/my_robot_node.cpp)
target_link_libraries(my_robot_node
  youbot_control
  rclcpp::rclcpp
)

ament_target_dependencies(my_robot_node
  rclcpp
  webots_ros2_driver
)

install(TARGETS my_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
