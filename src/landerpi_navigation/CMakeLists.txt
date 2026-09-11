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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_pytests test TIMEOUT 60)
endif()

ament_package()
