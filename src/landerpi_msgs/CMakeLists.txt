cmake_minimum_required(VERSION 3.8)
project(landerpi_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationPointState.idl"
  "msg/NavigationTaskState.idl"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_navigation_task_interfaces
    test/test_navigation_task_interfaces.py
  )
endif()

ament_package()
