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

Verify that in the terminal you are inside ros2_ws/src directory.

Python: ament_python, C++: ament_cmake

- ros2 pkg create name_of_the_package --build_type ament_cmake --dependencies dependency_name

Every time a new package is created, you need to source the workspace again, or open and close the terminal.

## Compiling

Inside ros2_ws directory:

- colcon build
- colcon build --packages-select name_of_the_package

## Run a node

- ros2 run name_of_the_package name_of_the_executable
- ros2 run name_of_the_package name_of_the_executable --ros-args -r __node:=new_node_name

## Topics

- ros2 topic list
- ros2 topic pub /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic pub -1 /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic pub -r 1.0 /name_of_the_topic type/of/interface "{data: value}"
- ros2 topic echo /name_of_the_topic
- ros2 topic hz /name_of_the_topic
- ros2 topic info /name_of_the_topic