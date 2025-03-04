## How to build micro-ROS-agent

``` bash
sudo apt-get install -y build-essential
sudo apt install libtinyxml2-dev

mkdir -p microros_ws/
cd microros_ws/
git clone https://github.com/fishros/micro-ROS-Agent.git -b humble
git clone https://github.com/micro-ROS/micro_ros_msgs.git -b humble

# Build
cd microros_ws
rm -rf build install log
#  --merge-install to 
colcon build --merge-install --install-base /opt/ros/myros_install

# Run in serial mode
ros2 run micro_ros_agent micro_ros_agent serial -b 921600 --dev /dev/ttyUSB0 -v6
# Run in UDP mode
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

```