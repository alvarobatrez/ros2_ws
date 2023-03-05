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

add_executable(publisher_template src/publisher_template.cpp)
ament_target_dependencies(publisher_template rclcpp std_msgs)

add_executable(subscriber_template src/subscriber_template.cpp)
ament_target_dependencies(subscriber_template rclcpp std_msgs)

install(TARGETS
  publisher_template
  subscriber_template
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
