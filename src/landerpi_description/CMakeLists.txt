cmake_minimum_required(VERSION 3.8)
project(landerpi_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

