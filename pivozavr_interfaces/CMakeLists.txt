cmake_minimum_required(VERSION 3.8)
project(pivozavr_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/WheelInfo.msg" "msg/AuxInfo.msg")

ament_package()
