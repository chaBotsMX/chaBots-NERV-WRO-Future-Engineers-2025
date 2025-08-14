cmake_minimum_required(VERSION 3.16)
project(teensy_communication)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teensy_comm_node src/teensy_comm_node.cpp)

# Usa SOLO ament_target_dependencies para evitar el conflicto
ament_target_dependencies(teensy_comm_node rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS teensy_comm_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()

