cmake_minimum_required(VERSION 3.8)
project(teensy_communication)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teensy_comm_node src/teensy_comm_node.cpp)
ament_target_dependencies(teensy_comm_node rclcpp std_msgs)

install(TARGETS teensy_comm_node
  DESTINATION lib/${PROJECT_NAME})

# Instala launch/ solo si existe (evita fallos en 'install' si falta la carpeta)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
