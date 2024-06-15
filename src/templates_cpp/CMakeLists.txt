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
find_package(rclcpp_action REQUIRED)

add_executable(transmitter src/transmitter.cpp)
ament_target_dependencies(transmitter rclcpp std_msgs)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp custom_interfaces)

add_executable(client_async src/client_async.cpp)
ament_target_dependencies(client_async rclcpp custom_interfaces)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp custom_interfaces rclcpp_action)

install(TARGETS
  transmitter
  receiver
  server
  client
  client_async
  action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
