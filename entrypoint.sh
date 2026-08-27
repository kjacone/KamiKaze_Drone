#!/bin/bash
set -e

# --- Force ROS to bind/advertise on localhost ---
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://localhost:11311

# --- Clean up stale X server state ---
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

# --- Check if running with GUI support ---
if [ -n "$DISPLAY" ] || [ -n "$ENABLE_GUI" ]; then
    echo "Starting GUI support..."
    
    # Start Xvfb
    echo "Starting Xvfb on :1 ..."
    Xvfb :1 -screen 0 1280x800x24 &
    sleep 2
    
    # Verify Xvfb is running
    if ! DISPLAY=:1 xdpyinfo >/dev/null 2>&1; then
        echo "WARNING: Xvfb failed to start. Continuing without GUI..."
    else
        DISPLAY=:1 fluxbox &
        x11vnc -display :1 -forever -nopw -quiet &
        websockify --web=/usr/share/novnc 8080 localhost:5900 &
        sleep 2
        echo "GUI available at http://localhost:8080/vnc.html"
    fi
fi

# --- Source ROS and build the catkin workspace ---
source /opt/ros/noetic/setup.bash

cd /root/catkin_ws

# Ensure the src directory exists
if [ ! -d src ]; then
    mkdir -p src
fi

# Check for drone_control package
if [ ! -d src/drone_control ]; then
    echo "WARNING: drone_control source not found in src/"
    echo "Looking for drone_control in mounted volumes..."
    
    # Try to find drone_control
    if [ -d /root/drone_control ]; then
        ln -s /root/drone_control src/drone_control
        echo "Found drone_control at /root/drone_control"
    elif [ -d /root/catkin_ws/src/drone_control ]; then
        echo "drone_control already in correct location"
    else
        echo "ERROR: drone_control source not found. Please mount it to /root/catkin_ws/src/drone_control"
        echo "Current directory contents:"
        ls -la /root/catkin_ws/src/ || echo "src directory empty"
        exit 1
    fi
fi

# Build the workspace
echo "Building catkin workspace..."
catkin_make || {
    echo "ERROR: catkin_make failed"
    echo "Checking for ROS package dependencies..."
    rosdep install --from-paths src --ignore-src -r -y || true
    catkin_make
}
source devel/setup.bash

# Verify the package is found
if ! rospack find drone_control >/dev/null 2>&1; then
    echo "ERROR: drone_control package not found after catkin_make."
    echo "Check that ./drone_control on the host contains a valid package.xml"
    exit 1
fi

echo "drone_control package found at: $(rospack find drone_control)"

# --- Set up PX4 Autopilot ---
if [ -d /root/PX4-Autopilot ]; then
    cd /root/PX4-Autopilot
    
    # Install Python requirements if needed
    if [ -f Tools/setup/requirements.txt ]; then
        pip3 install -r Tools/setup/requirements.txt >/dev/null 2>&1 || true
    fi
    
    # Build PX4 only if not already built
    if [ ! -f build/px4_sitl_default/bin/px4 ]; then
        echo "Building PX4 SITL (first time, this may take a while)..."
        DONT_RUN=1 make px4_sitl_default gazebo-classic_iris || {
            echo "WARNING: PX4 build failed. Continuing without PX4..."
        }
    else
        echo "PX4 SITL already built, skipping build."
    fi
    
    # Source PX4 Gazebo environment
    if [ -f Tools/simulation/gazebo-classic/setup_gazebo.bash ]; then
        source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
    fi
fi

# --- Launch the simulation stack (if not in test mode) ---
if [ "$SKIP_SIMULATION" != "true" ]; then
    echo "Launching PX4 SITL..."
    roslaunch px4 posix_sitl.launch &
    PX4_PID=$!
    sleep 10
    
    # Check if PX4 is still running
    if ! kill -0 $PX4_PID 2>/dev/null; then
        echo "WARNING: PX4 SITL failed to start or crashed"
        PX4_PID=""
    fi
    
    echo "Launching MAVROS..."
    roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 &
    MAVROS_PID=$!
    sleep 5
    
    if ! kill -0 $MAVROS_PID 2>/dev/null; then
        echo "WARNING: MAVROS failed to start or crashed"
        MAVROS_PID=""
    fi
fi

# --- Launch drone_control nodes ---
echo "Launching drone_control nodes..."

# Set the launch file path
LAUNCH_FILE="/root/catkin_ws/src/drone_control/launch/kamikaze.launch"

# Verify the launch file exists
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "ERROR: kamikaze.launch not found at $LAUNCH_FILE"
    echo "Looking for launch files..."
    find /root/catkin_ws/src/drone_control -name "*.launch" || echo "No launch files found"
    exit 1
fi

# Determine launch mode
if [ -n "$TEST_MODE" ]; then
    LAUNCH_MODE="mode:=test test_scenario:=default"
elif [ -n "$DEBUG_MODE" ]; then
    LAUNCH_MODE="mode:=debug debug_level:=debug"
else
    LAUNCH_MODE="mode:=production"
fi

# Launch with error handling
echo "Launching with: roslaunch drone_control kamikaze.launch $LAUNCH_MODE"
if ! roslaunch drone_control kamikaze.launch $LAUNCH_MODE; then
    echo "ERROR: drone_control launch failed"
    echo "Checking launch file syntax..."
    cat "$LAUNCH_FILE" | head -20
    exit 1
fi &

DRONE_CONTROL_PID=$!

# Wait a moment for the nodes to initialize
sleep 5

# Check if the roslaunch process is still running
if ! kill -0 $DRONE_CONTROL_PID 2>/dev/null; then
    echo "ERROR: drone_control launch process terminated unexpectedly"
    echo "Check ROS logs:"
    tail -100 /root/.ros/log/latest/*.log 2>/dev/null || echo "No logs found"
    exit 1
fi

# --- Display status ---
echo ""
echo "=========================================="
echo "  🚀 KAMIKAZE DRONE SYSTEM STARTED"
echo "=========================================="
echo "  Mode: $([ -n "$TEST_MODE" ] && echo "TEST" || ([ -n "$DEBUG_MODE" ] && echo "DEBUG" || echo "PRODUCTION"))"
echo "  PX4 SITL: $([ -n "$PX4_PID" ] && echo "RUNNING" || echo "SKIPPED")"
echo "  MAVROS: $([ -n "$MAVROS_PID" ] && echo "RUNNING" || echo "SKIPPED")"
echo "  Drone Control: RUNNING (PID: $DRONE_CONTROL_PID)"
echo "  GUI: $([ -n "$DISPLAY" ] || [ -n "$ENABLE_GUI" ] && echo "ENABLED at http://localhost:8080/vnc.html" || echo "DISABLED")"
echo "=========================================="
echo ""

# --- Keep the container alive with proper process monitoring ---
cleanup() {
    echo ""
    echo "Shutting down processes..."
    kill $DRONE_CONTROL_PID 2>/dev/null || true
    kill $MAVROS_PID 2>/dev/null || true
    kill $PX4_PID 2>/dev/null || true
    echo "Cleanup complete"
    exit 0
}

trap cleanup SIGTERM SIGINT

# Monitor processes and keep container alive
while true; do
    # Check if critical processes are still running
    if [ -n "$PX4_PID" ] && ! kill -0 $PX4_PID 2>/dev/null; then
        echo "WARNING: PX4 SITL process died"
        PX4_PID=""
    fi
    
    if [ -n "$MAVROS_PID" ] && ! kill -0 $MAVROS_PID 2>/dev/null; then
        echo "WARNING: MAVROS process died"
        MAVROS_PID=""
    fi
    
    if ! kill -0 $DRONE_CONTROL_PID 2>/dev/null; then
        echo "ERROR: drone_control launch process died. Shutting down..."
        cleanup
    fi
    
    sleep 10
done