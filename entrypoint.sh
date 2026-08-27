#!/bin/bash
set -e

# --- Force ROS to bind/advertise on localhost so it can reach itself
#     under host networking, where the "kamikaze" hostname may not
#     resolve back to this container. ---
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://localhost:11311

# --- Clean up stale X server state from a previous run of this container ---
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# --- Start GUI ---
echo "Starting Xvfb on :1 ..."
Xvfb :1 -screen 0 1280x800x24 &
sleep 2
# Verify Xvfb is running
if ! DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
    echo "ERROR: Xvfb failed to start on :1 — check /tmp/.X1-lock or a stale process"
    exit 1
fi

DISPLAY=:1 fluxbox &
x11vnc -display :1 -forever -nopw -quiet &
websockify --web=/usr/share/novnc 8080 localhost:5900 &
sleep 2

# --- Source ROS and build the catkin workspace ---
source /opt/ros/noetic/setup.bash

cd /root/catkin_ws

# Ensure the src directory exists (if workspace is freshly mounted)
if [ ! -d src ]; then
    mkdir -p src
fi

# The drone_control package should be mounted at /root/catkin_ws/src/drone_control
# If it's not there, try to symlink it from the expected mount point (just in case)
if [ ! -d src/drone_control ]; then
    echo "WARNING: drone_control source not found in src/ — attempting to locate it..."
    # It might be mounted directly under /root/catkin_ws (not inside src), so check
    if [ -d /root/drone_control ]; then
        ln -s /root/drone_control src/drone_control
    elif [ -d /root/catkin_ws/src/drone_control ]; then
        : # already there
    else
        echo "ERROR: drone_control source not found. Please mount it to /root/catkin_ws/src/drone_control"
        exit 1
    fi
fi

# Build the workspace
echo "Building catkin workspace..."
catkin_make
source devel/setup.bash

# Verify the package is found
if ! rospack find drone_control >/dev/null 2>&1; then
    echo "ERROR: drone_control package not found after catkin_make."
    echo "Check that ./drone_control on the host contains a valid package.xml"
    echo "and is mounted at /root/catkin_ws/src/drone_control."
    exit 1
fi

# --- Set up PX4 Autopilot ---
cd /root/PX4-Autopilot

# Install Python requirements if needed (idempotent)
if [ -f Tools/setup/requirements.txt ]; then
    pip3 install -r Tools/setup/requirements.txt >/dev/null 2>&1 || true
fi

# Build PX4 only if not already built (speeds up subsequent starts)
if [ ! -f build/px4_sitl_default/bin/px4 ]; then
    echo "Building PX4 SITL (first time, this may take a while)..."
    DONT_RUN=1 make px4_sitl_default gazebo-classic_iris
else
    echo "PX4 SITL already built, skipping build."
fi

# Source PX4 Gazebo environment
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# --- Launch the simulation stack ---
echo "Launching PX4 SITL..."
roslaunch px4 posix_sitl.launch &
sleep 10

echo "Launching MAVROS..."
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 &
sleep 5

echo "Launching drone_control nodes..."
rosrun drone_control vehicle_state_monitor.py &
rosrun drone_control target_tracking_controller.py &

# --- Keep the container alive ---
echo "All services started. Container is running."
tail -f /dev/null