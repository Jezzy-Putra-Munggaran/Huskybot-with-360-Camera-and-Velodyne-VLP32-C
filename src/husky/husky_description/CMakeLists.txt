cmake_minimum_required(VERSION 3.5)
project(husky_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()