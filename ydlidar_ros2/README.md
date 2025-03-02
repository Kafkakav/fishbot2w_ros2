
### Install ROS2 Humble developement packages
[Ubuntu Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
``` bash
# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # check for UTF-8

# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install software-properties-common
sudo add-apt-repository universe

#Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools

```

### Environment setup

``` bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

# try some examples  - Talker-listener
# terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
# terminal 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener

```


### change the used yaml file ydlidar.yaml for your environment 
``` yaml
ydlidar_node:
  ros__parameters:
    port: /dev/ttyUSB0  # <<< HERE, /tmp/fishbot_laser for fishbor-laser-driver(wifi<->uart)
    frame_id: laser_frame
    ignore_array: ""

```

### Build from source
``` bash
cd ydlidar_ros2
colcon build

```

### run ROS2 node ydlidar 
``` bash
sudo chmod 666 /dev/ttyUSB0

# run fishbot-laser-drver if you used /tmp/fishbot_laser, 

source install/setup.bash
ros2 launch ydlidar ydlidar_launch.py

ros2 topic list
# /parameter_events
# /rosout
# /scan
# /tf_static
# /ydlidar_node/transition_event
```

### RVIZ2
[可视化雷达点云-学会驱动雷达] (https://fishros.com/d2lros2/#/humble/chapt17/slam/1.%E5%8F%AF%E8%A7%86%E5%8C%96%E9%9B%B7%E8%BE%BE%E7%82%B9%E4%BA%91-%E5%AD%A6%E4%BC%9A%E9%A9%B1%E5%8A%A8%E9%9B%B7%E8%BE%BE)
``` bash
source /opt/ros/humble/setup.bash

# run command on GUI display mode
rviz2
```
<img src="docs/pics/ros2_rviz2.png" alt="ros2_rviz2.png" style="zoom: 25%;" />

 