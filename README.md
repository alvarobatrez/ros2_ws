# ROS2 Templates

## Install ROS 2 Humble

Follow the instructions in the documentation page. After finishing also install in the terminal:

- sudo apt install python3-colcon-common-extensions
- sudo apt install python3-pip
- pip3 install setuptools==58.2.0

In the terminal type gedit ~/.bashrc and write the following at the end of the file:

- source /opt/ros/humble/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
- source ~/ros2_ws/install/setup.bash

ros2_ws is a common name for the workspace but it can be anyone.

Save the file and close the terminal.

## Create a package

Verify in the terminal that you are inside ros2_ws/src directory.

- ros2 pkg create name_of_the_package --build_type ament_cmake --dependencies dependency_1_name dependency_2_name

Python: ament_python, C++: ament_cmake

Examples:

- ros2 pkg create my_package --build_type --ament_python --dependencies rclpy std_msgs my_interface

- ros2 pkg create my_package --build_type --ament_cmake --dependencies rclcpp std_msgs my_interface

Every time a new package is created, you need to source the workspace again, or close and open the terminal.

## Compiling

Inside ros2_ws directory:

- colcon build
- colcon build --packages-select name_of_the_package

## Run a node

- ros2 run name_of_the_package name_of_the_executable
- ros2 run name_of_the_package name_of_the_executable --ros-args -r __node:=new_node_name
- ros2 run name_of_the_package name_of_the_executable --ros-args -r /name_of_the_topic:=/new_name_of_the_topic
- ros2 run name_of_the_package name_of_the_executable --ros-args -r /name_of_the_service:=/new_name_of_the_service

## Topics

- ros2 topic list
- ros2 topic pub /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic pub -1 /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic pub -r 1.0 /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic echo /name_of_the_topic
- ros2 topic hz /name_of_the_topic
- ros2 topic info /name_of_the_topic

## Services

- ros2 service list
- ros2 service call /name_of_the_service type/of/interface "{request: data}"

## Parameters

- ros2 param list
- ros2 param get /name_of_the_node name_of_the_param