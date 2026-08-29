#!/bin/bash
set -e

# ROS Environment
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_IP=${ROS_IP:-127.0.0.1}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

# Clean up stale X server state
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# Source ROS
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Start GUI if enabled
if [ -n "$DISPLAY" ] || [ -n "$ENABLE_GUI" ]; then
    echo "Starting GUI support..."
    Xvfb :1 -screen 0 1280x800x24 &
    sleep 2
    DISPLAY=:1 fluxbox &
    x11vnc -display :1 -forever -nopw -quiet &
    websockify --web=/usr/share/novnc 8080 localhost:5900 &
    echo "GUI available at http://localhost:8080/vnc.html"
fi

# Set Python path
export PYTHONPATH=/root/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH

# Launch simulation if enabled
if [ "${ENABLE_SIMULATION:-false}" = "true" ]; then
    echo "Starting PX4 SITL simulation..."
    # Note: PX4 SITL needs to be built separately
    # cd /px4 && make px4_sitl gazebo
    # roslaunch px4 posix_sitl.launch &
    # sleep 5
    roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 &
    sleep 5
fi

# Execute command
if [ $# -eq 0 ]; then
    # Default launch
    exec roslaunch drone_control kamikaze.launch mode:=production
else
    exec "$@"
fi