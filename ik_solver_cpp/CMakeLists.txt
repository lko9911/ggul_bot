cmake_minimum_required(VERSION 3.8)
project(ik_solver_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(ik_subscriber_node src/ik_subscriber_node.cpp)
ament_target_dependencies(ik_subscriber_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
)

install(TARGETS ik_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# ⬇️ launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

