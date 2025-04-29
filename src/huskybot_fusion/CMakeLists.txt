cmake_minimum_required(VERSION 3.8)
project(huskybot_fusion)

# Temukan dependency utama
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolov12_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object3D.msg"
  DEPENDENCIES std_msgs
)

# Install Python module
ament_python_install_module(${PROJECT_NAME})

# Install Python scripts (jika ada)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install msg files (untuk referensi)
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()