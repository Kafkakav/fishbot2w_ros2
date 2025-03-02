### 通過Web頁面實現一個簡單的ROS遙控器 - rosbridge_server
We can control a ros-robot to move via rosbridge_websocket
``` bash
source /opt/ros/humble/setup.sh

# install dependencies
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
# launch ros2 websocket server 
ros2 run rosbridge_server rosbridge_websocket

```

### RobotWebTools - roslibjs
The Standard ROS JavaScript Library
```
https://github.com/RobotWebTools/roslibjs

```

### Basic web server in Flask
``` bash
pip install Flask

python webmain.py
```