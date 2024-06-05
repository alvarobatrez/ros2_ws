cmake_minimum_required(VERSION 3.8)
project(templates_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(transmitter src/transmitter.cpp)
ament_target_dependencies(transmitter rclcpp std_msgs)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)

install(TARGETS
  transmitter
  receiver
  server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
