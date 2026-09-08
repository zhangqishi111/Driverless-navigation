cmake_minimum_required(VERSION 3.8)
project(landerpi_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_pose_bridge.py
  scripts/goal_bridge.py
  scripts/localization_safety_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

