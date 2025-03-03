#!/bin/bash
#
export WORKING_DIR=${PWD}

source /opt/ros/humble/setup.sh

error_exit() {
  echo -e "\033[91m    ----------------------------------------------    \033[0m"
  echo -e "\033[91m    !!!!! ERROR: ${1:-"Unknown Error"}          \033[0m"
  echo -e "\033[91m    ----------------------------------------------    \033[0m"
  exit 1
}

kill_process_by_pidfile() {
    local PID_FILE="$1" 

    # 檢查 PID 文件是否存在
    if [ ! -f "$PID_FILE" ]; then
        echo "PID file $PID_FILE does not exist. Exiting."
        return 1
    fi

    # 讀取 PID 文件中的進程 ID
    local PID
    PID=$(cat "$PID_FILE")

    # 檢查進程是否存在
    if ps -p "$PID" > /dev/null 2>&1; then
        echo "Process with PID $PID is running. Killing it..."
        kill "$PID"
        
        # 等待 2 秒，檢查進程是否終止
        sleep 2
        if ps -p "$PID" > /dev/null 2>&1; then
            echo "Process with PID $PID is still running. Force killing it..."
            kill -9 "$PID"
        fi
        
        echo "Process with PID $PID has been killed."
    else
        echo "Process with PID $PID is not running."
    fi

    # 刪除 PID 文件
    # rm -f "$PID_FILE"
    # echo "PID file $PID_FILE has been removed."
}

get_pid_by_cmdline() {
  local command_line="$1"
  local pid

  pid=$(pgrep -f "$command_line")

  if [ -n "$pid" ]; then
    echo "$pid"
  else
    return 1 # Indicate failure
  fi
}

kill_process_by_cmdline() {
  local command_line="$1" 
  pid=$(pgrep -f "$command_line")
  
  if [ -n "$pid" ]; then
    echo "Kill PID for command: $command_line is: $pid"
    kill -2 $pid
    sleep 1
    kill -9 $pid
  fi
}

##################################################
# laser_driver(Wi-Fi)
# Wi-Fi tcp:8889 <=> virtual UART /tmp/fishbot_laser
start_wifi_laser_driver() {
  FISHBOT_LASER_X2_DIR=${WORKING_DIR}/ydlidar_ros2/fishbot_laser_driver
  FISHBOT_LASER_X2_PID="${FISHBOT_LASER_X2_DIR}/laser_driver.pid"
  FISHBOT_LASER_X2_EXE="${FISHBOT_LASER_X2_DIR}/fishbot_laser_x2.py"
  FISHBOT_LASER_X2_LOG="${FISHBOT_LASER_X2_DIR}/fishbot_laser_out.log"
  
  kill_process_by_pidfile "${FISHBOT_LASER_X2_PID}"
  python ${FISHBOT_LASER_X2_EXE} > ${FISHBOT_LASER_X2_LOG} 2>&1 &
}

##################################################
# ydlidar_ros2_node: /ydlidar_node
start_laser_ros_node() {
  YDLIDAR_ROS_NODE_DIR=${WORKING_DIR}/ydlidar_ros2
  source ${YDLIDAR_ROS_NODE_DIR}/install/setup.bash
  ros2 node list
  kill_process_by_cmdline "/ydlidar/lib/ydlidar/ydlidar_node --ros-args"
  sleep 1
  ros2 launch ydlidar ydlidar_launch.py &
}

##################################################
# micro-ros-agent-humble: docker container udp:8888
start_uros_agent_ros_node() {
  UROS_AGENT_CONTAINER_NAME="microros_agent_humble"
  if docker inspect "${UROS_AGENT_CONTAINER_NAME}" > /dev/null 2>&1; then
      echo "Container ${UROS_AGENT_CONTAINER_NAME} exists."
  else
      echo "Run Container ${UROS_AGENT_CONTAINER_NAME}..."
      docker run -dt --name microros_agent_humble --rm -v /dev:/dev -v /dev/shm:/dev/shm \
          --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v4
  fi
}
##################################################
# teleop_twist_web
# ROS2-Web websocket bridge: /rosbridge_websocket
start_rosbridge_websocket() {
  pid_result=$(get_pid_by_cmdline "python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocke")
  return_code=$?
  echo "Return code: $return_code"
  if [ $return_code -eq 0 ]; then
    echo "rosbridge_websocket PID found: $pid_result"
  else
    echo "rosbridge_websocket is starting"
    ros2 run rosbridge_server rosbridge_websocket &
  fi
}

start_teleop_web_node() {
  TELEOP_TWIST_WEB_DIR=${WORKING_DIR}/teleop_twist_web
  TELEOP_TWIST_WEB_EXE="${TELEOP_TWIST_WEB_DIR}/webmain.py"
  TELEOP_TWIST_WEB_PID="${TELEOP_TWIST_WEB_DIR}/teleop_web.pid"
  TELEOP_TWIST_WEB_LOG="${TELEOP_TWIST_NODE_DIR}/teleop_web_out.log"
  pid_result=$(get_pid_by_cmdline "python3 ${TELEOP_TWIST_WEB_EXE}")
  return_code=$?
  if [ $return_code -eq 0 ]; then
    echo "teleop_twist_web PID found: $pid_result"
  else
    echo "teleop_twist_web is starting"
    python3 ${TELEOP_TWIST_WEB_EXE} > ${TELEOP_TWIST_WEB_LOG} 2>&1 &
  fi
}

##################################################

fishros_menu() {
local bMebuSelected=
local inpItem
while [ -z "$bMebuSelected" ]
do
  echo ""
  echo "----------------------------------------------------------"  
  echo "ROS2 Management Menu: "
  echo -e "\033[93m  1. 雷達傳輸板ROS(incl. laser-driver)  \033[0m"
  echo -e "\033[93m  2. 驅動主板ROS(micor-agent) \033[0m"
  echo -e "\033[93m  3. 網頁版控制器(incl. rosbridge)  \033[0m"
  echo -e "\033[93m  0. 離開 Quit \033[0m"
  echo -e "Individual tasks:"
  echo -e "\033[93m  a. start_wifi_laser_driver  \033[0m"
  echo -e "\033[93m  b. start_laser_ros_node  \033[0m"
  echo -e "\033[93m  c. start_uros_agent_ros_node  \033[0m"
  echo -e "\033[93m  d. start_rosbridge_websocket  \033[0m"
  echo -e "\033[93m  e. start_teleop_web_node  \033[0m"
  echo -n "Which item would you start? "

    if [ -z "$1" ] ; then
        read inpItem
    fi
    echo ">>>>> "

    case "$inpItem" in
    "0")
        bMebuSelected="Quit";
        exit 0
        ;;
    "1")
        start_wifi_laser_driver "p1"
        start_laser_ros_node "p1"
        ;;
    "2")
        start_uros_agent_ros_node "p1"
        ;;
    "3")
        start_rosbridge_websocket "p1"
        start_teleop_web_node "p1"
        ;;        
    "a")
        start_wifi_laser_driver "p1"
        ;;
    "b")
        start_laser_ros_node "p1"
        ;;
    "c")
        start_uros_agent_ros_node "p1"
        ;;
    "d")
        start_rosbridge_websocket "p1"
        ;;
    "e")
        start_teleop_web_node "p1"
        ;;
    *)
        echo -n "ERROR: Unknown item: $inpItem"
        ;;
    esac
done
}

fishros_menu