cmake_minimum_required(VERSION 3.8)
project(landerpi_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_sim_all.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
