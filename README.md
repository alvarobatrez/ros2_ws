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