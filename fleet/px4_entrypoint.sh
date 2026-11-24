#!/bin/bash
# PX4 SITL launcher for one fleet drone.
#
# Mirrors the stock image entrypoint but adds `px4 -i <instance>` so each
# drone gets its own MAVLink ports (14580+instance / 14540+instance) even
# though all MAVROS instances share the ROS container's network namespace.
#
# Args: <MAVROS_IP> <QGC_IP>; both are passed explicitly to keep the image's
# host.docker.internal fallback out of the picture.
set -e

: "${PX4_INSTANCE:?PX4_INSTANCE must be set}"

Xvfb :99 -screen 0 1600x1200x24+32 &
"${SITL_RTSP_PROXY}/build/sitl_rtsp_proxy" &

# Stream the Gazebo camera to the ROS container (TCP; gz capture is halved
# to 640x480) and drop the two scenario props into the world once it is up.
python3 /root/gz_camera_bridge.py \
    --topic "/world/default/model/x500_mono_cam_${PX4_INSTANCE}/link/camera_link/sensor/imager/image" \
    --host "$1" --port "${GZ_CAM_PORT:-9200}" >/tmp/gz_camera_bridge.log 2>&1 &
python3 /root/gz_scenario.py >/tmp/gz_scenario.log 2>&1 &

source "${WORKSPACE_DIR}/edit_rcS.bash" "$1" "$2"

cd "${FIRMWARE_DIR}"

exec env HEADLESS=1 PX4_SIM_MODEL=gz_x500_mono_cam PX4_GZ_WORLD=default \
    "${FIRMWARE_DIR}/build/bin/px4" -i "${PX4_INSTANCE}" &
PX4_PID=$!

# Watchdog: Gazebo can die without taking PX4 down, leaving the drone
# connected to a simulator that no longer exists. Exit non-zero so the
# container's restart policy recreates the sim.
gz_seen=0
gz_misses=0
start_ts=$(date +%s)
while kill -0 "$PX4_PID" 2>/dev/null; do
    sleep 5
    if pgrep -f 'gz sim' >/dev/null 2>&1; then
        gz_seen=1
        gz_misses=0
    elif [ "$gz_seen" = 1 ]; then
        gz_misses=$((gz_misses + 1))
        if [ "$gz_misses" -ge 3 ]; then
            echo 'watchdog: Gazebo disappeared; restarting container' >&2
            kill -9 "$PX4_PID" 2>/dev/null
            exit 1
        fi
    elif [ $(( $(date +%s) - start_ts )) -gt 180 ]; then
        echo 'watchdog: Gazebo never started; restarting container' >&2
        kill -9 "$PX4_PID" 2>/dev/null
        exit 1
    fi
done
wait "$PX4_PID" || true
exit 1
