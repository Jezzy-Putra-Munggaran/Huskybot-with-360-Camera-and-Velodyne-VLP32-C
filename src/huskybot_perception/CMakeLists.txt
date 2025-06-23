cmake_minimum_required(VERSION 3.5)
project(huskybot_perception)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolov12_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  ${PROJECT_NAME}/fusion_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Create symlinks for executable scripts
install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/fusion_visualizer_node.py
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/fusion_visualizer_node
)")

ament_package()