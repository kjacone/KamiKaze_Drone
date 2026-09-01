#!/bin/bash

set -e

echo "============================================"
echo " Starting  Drone Simulation Container"
echo "============================================"

# ------------------------------------------------------------
# ROS Environment
# ------------------------------------------------------------

export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

echo "ROS_HOSTNAME=${ROS_HOSTNAME}"
echo "ROS_IP=${ROS_IP}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"

# ------------------------------------------------------------
# Source ROS
# ------------------------------------------------------------

source /opt/ros/noetic/setup.bash

if [ -f /root/catkin_ws/devel/setup.bash ]; then
    source /root/catkin_ws/devel/setup.bash
fi

# ------------------------------------------------------------
# Python
# ------------------------------------------------------------

export PYTHONUNBUFFERED=1

export PYTHONPATH="/root/catkin_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

# ------------------------------------------------------------
# Clean stale X11 state
# ------------------------------------------------------------

rm -f /tmp/.X1-lock
rm -f /tmp/.X11-unix/X1

# ------------------------------------------------------------
# GUI
# ------------------------------------------------------------

if [ "${ENABLE_GUI:-false}" = "true" ]; then

    echo "Starting GUI support..."

    export DISPLAY=:1

    Xvfb :1 -screen 0 1280x800x24 &

    sleep 2

    fluxbox &

    x11vnc \
        -display :1 \
        -forever \
        -nopw \
        -quiet &

    websockify \
        --web=/usr/share/novnc \
        8080 \
        localhost:5900 &

    echo "GUI available at:"
    echo "http://localhost:8080/vnc.html"

else

    echo "GUI disabled."

fi

# ------------------------------------------------------------
# Simulation
# ------------------------------------------------------------

if [ "${ENABLE_SIMULATION:-false}" = "true" ]; then

    echo "Simulation mode enabled."

    # Start only the simulation components that are
    # actually available in this image.
    #
    # Keep simulator startup separate from the ROS
    # application so failures are easier to diagnose.

    if command -v roscore >/dev/null 2>&1; then

        echo "Checking ROS master..."

        if ! rosnode list >/dev/null 2>&1; then
            echo "Starting ROS master..."

            roscore > /tmp/roscore.log 2>&1 &

            ROSCORE_PID=$!

            sleep 5

            if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
                echo "ERROR: roscore failed to start."
                cat /tmp/roscore.log
                exit 1
            fi
        fi

    fi

fi

# ------------------------------------------------------------
# Default command
# ------------------------------------------------------------

if [ "$#" -eq 0 ]; then

    echo "Starting default ROS application..."

    exec roslaunch drone_control kamikaze.launch mode:=production

else

    echo "Executing custom command:"
    echo "$@"

    exec "$@"

fi