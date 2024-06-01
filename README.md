# ROS 2

## Install

### VirtualBox

If you prefer to use VirtualBox instead of a dedicated partition follow these steps.

- Download VirtualBox and install it.

- Download Ubuntu 24.04 LTS.

- Create a new virtual environment, select the iso, uncheck the unattended installation, select at leat 2 cores and 4096 mb, select NAT network adapter and bidirectional clipboard.

- Follow the installation steps and restart.

#### Fullscreen

Open the terminal and type:

```
sudo apt install build-essential gcc make perl dkms
```

Insert guest image, open it and open the terminal with right-click and type:

```
./autorun.sh
```

Restart the virtual machine and resize the screen.

#### Shared folder

Open the terminal and type:

```
sudo adduser your_vm_username vboxsf
```

Shutdown the VM. In configuration select the shared folder path.

### Tools (optional)

```
sudo apt install terminator
sudo snap install code --classic
```

### ROS

Open the terminal and type:

```
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
sudo apt install cmake git python3-colcon-common-extensions python3-flake8 python3-rosdep python3-setuptools python3-vcstool wget
```

Type:

```
sudo rosdep init
```

and follow the instructions.

Close the terminal, open it again and type:

```
gnome-text-editor .bashrc
```

At the end of the file copy the next lines:

```
source /opt/ros/jazzy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```

Save the file, close it and close the terminal.

## Create a workspace

Open the terminal and type:

```
mkdir -p ros2_ws/src
```

Note: a common name for the workspace is ros2_ws but it can be whatever you want, it just needs to be the same name as in the .bashrc file.

## Create a package

Move to the src directory:

```
cd ros2_ws/src
```

Create a Python package:

```
ros2 pkg create name_of_the_package --build_type ament_python --dependencies rclpy dependency_1 dependency_2
```

Create a C++ package:

```
ros2 pkg create name_of_the_package --build_type ament_cmake --dependencies rclcpp dependency_1 dependency_2
```

Create a custom package:

```
ros2 pkg create name_of_the_package --dependencies std_msgs action_msgs dependency_1 dependency_2
```

Note 1: action_msgs is just required if you want to use actions.

Note 2: in all cases above, dependencies are optional.

## Compile

Inside ros2_ws compile all packages with:

```
colcon build
```

or just a selected package:

```
colcon build --packages-select name_of_the_package
```

Note: every time you create a new package you need to resource with `source ~/ros2_ws/install/setup.bash` or relaunch the terminal.

## Nodes

### Terminal Commands

```
ros2 run name_of_the_package name_of_the_executable
ros2 run name_of_the_package name_of_the executable --ros-args -r __name:=new_node_name
ros2 node list
ros2 node info /name_of_the_node
```

## Topics

### Terminal Commands

```
ros2 topic list
ros2 topic info /name_of_the_topic
ros2 topic echo /name_of_the_topic
ros2 topic hz /name_of_the_topic
ros2 topic pub -1 /name_of_the_topic name_of_the_interface "{data: value}"
ros2 topic pub --rate 1 /name_of_the_topic name_of_the_interface "{data: value}"
```

## Services

### Terminal Commands

```
ros2 service list
ros2 service type /name_of_the_service
ros2 service call /name_of_the_service name_of_the_interface "{request: value}
```

## Parameters

### Terminal Commands

```
ros2 run name_of_the_package name_of_the_executable --ros-args -p parameter:=new_value
ros2 param list
ros2 param set /name_of_the_node name_of_the_parameter value
ros2 param get /name_of_the_node name_of_the_parameter
```

## Actions

### Terminal Commands

```
ros2 action list
ros2 action info
ros2 action send_goal /name_of_the_action name_of_the_interface "{goal: value}"
ros2 action send_goal -f /name_of_the_action name_of_the_interface "{goal: value}"
ros2 service call /name_of_the_action/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

## Launch

### Terminal Commands

```
ros2 launch name_of_the_launch_pkg name.launch.py
```
