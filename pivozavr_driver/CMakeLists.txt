cmake_minimum_required(VERSION 3.8)
project(pivozavr_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pivozavr_interfaces REQUIRED)

include_directories("include/io")
include_directories("include/protocol")

add_executable(${PROJECT_NAME} src/main.cpp include/io/SerialConnector.hpp src/io/SerialConnector.cpp include/protocol/ModbusMaster.hpp src/protocol/ModbusMaster.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp pivozavr_interfaces)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
