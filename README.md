# Autonomous Kamikaze Drone Project

> Educational and research use only.
>
> This project is intended for controlled simulation environments. It is not
> recommended for real-world surveillance, tracking, weaponization, or
> operational deployment. Users are responsible for complying with all
> applicable laws, safety requirements, and organizational policies.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Key Components](#key-components)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Project](#running-the-project)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Mission States](#mission-states)
- [Contributing](#contributing)
- [License](#license)
- [Disclaimer](#disclaimer)

## Overview

This repository contains a ROS-based autonomous drone simulation that combines
YOLOv8 object detection, MAVROS, PX4 SITL, and Gazebo Classic. The project is
structured for experimenting with simulated perception, vehicle state
monitoring, and target-tracking control loops.

## System Architecture

```text
Camera / Detection (YOLOv8)
        |
        v
/detected_objects
        |
        v
Target Tracking Controller
        |
        +-- /mavros/setpoint_raw/local
        |
        +-- /mavros/setpoint_velocity/cmd_vel_unstamped
                         |
                         v
                       MAVROS
                         |
                         v
                        PX4
                         |
                         v
                       Gazebo
                         |
                         +-- Position / IMU feedback
```

## Key Components

| Component | Description | Interface |
| --- | --- | --- |
| YOLO Detector | Detects supported object classes in image streams. | Publishes `/detected_objects`. |
| Target Tracker | Runs the target-tracking state machine. | Subscribes to `/detected_objects` and `/mavros/local_position/odom`. |
| Vehicle State Monitor | Monitors vehicle state from MAVROS. | Reads MAVROS state and odometry topics. |
| PX4 SITL | Simulated flight controller. | Runs PX4 software-in-the-loop. |
| MAVROS | ROS-to-PX4 communication bridge. | Exposes MAVLink data as ROS topics and services. |
| Gazebo Classic | 3D simulation environment. | Provides simulated world and sensor feedback. |

## Prerequisites

### Docker Setup

The Docker workflow is recommended for a consistent local environment.

- Docker Engine 20.10 or newer
- Docker Compose 1.29 or newer, or the Docker Compose v2 plugin
- 8 GB RAM minimum
- 16 GB RAM recommended
- 20 GB free disk space or more

### Native Setup

- Ubuntu 20.04 LTS
- ROS Noetic
- PX4 Autopilot
- MAVROS and MAVROS Extras
- Gazebo Classic
- Python 3
- NVIDIA GPU optional, for faster YOLO inference

## Installation

### Method 1: Docker

Clone the repository:

```bash
git clone https://github.com/kjacone/KamiKaze_Drone.git
cd KamiKaze_Drone
```

Build the Docker image:

```bash
docker-compose build
```

Start the container:

```bash
docker-compose up -d
```

Verify that the container is running:

```bash
docker ps | grep kamikaze_drone
```

View logs:

```bash
docker logs kamikaze_drone
```

Open the simulation in a browser:

```text
http://localhost:8080/vnc.html
```

### Method 2: Native Ubuntu Installation

Install ROS Noetic:

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install MAVROS:

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

Install PX4 Autopilot:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
```

Install Gazebo ROS packages:

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

Install Python dependencies:

```bash
pip3 install -r requirements.txt
```

Clone this repository:

```bash
cd ~
git clone https://github.com/kjacone/KamiKaze_Drone.git
cd KamiKaze_Drone
```

Set up the ROS workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/KamiKaze_Drone/drone_control .

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Copy the modified Iris model, if your setup requires it:

```bash
cp ~/KamiKaze_Drone/modified_px4/Tools/simulation/gazebo-classic/models/iris/iris.sdf.jinja \
  ~/PX4-Autopilot/Tools/simulation/gazebo-classic/models/iris/
```

## Running the Project

### Docker Mode

Start the container:

```bash
docker-compose up -d
```

Open noVNC:

```text
http://localhost:8080/vnc.html
```

Enter the running container:

```bash
docker exec -it kamikaze_drone bash
```

Source the ROS workspace inside the container:

```bash
source /root/catkin_ws/devel/setup.bash
```

Run the test detector:

```bash
rosrun drone_control test_detector.py
```

Run the YOLO detector:

```bash
rosrun drone_control yolo_detector.py
```

Monitor velocity commands:

```bash
rostopic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

Stop the container:

```bash
docker-compose down
```

### Native Mode

Open five terminals and run the following commands.

#### Terminal 1: Launch PX4 SITL

```bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic_iris

source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  "$(pwd)" \
  "$(pwd)/build/px4_sitl_default"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch px4 posix_sitl.launch
```

#### Terminal 2: Launch MAVROS

```bash
source ~/catkin_ws/devel/setup.bash

roslaunch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

#### Terminal 3: Start Vehicle State Monitor

```bash
source ~/catkin_ws/devel/setup.bash
rosrun drone_control vehicle_state_monitor.py
```

#### Terminal 4: Start Target Tracking Controller

```bash
source ~/catkin_ws/devel/setup.bash
rosrun drone_control target_tracking_controller.py
```

#### Terminal 5: Visualize Processed Images

Start `rqt`:

```bash
rqt
```

Then open `Plugins > Visualization > Image View` and select:

```text
/processed_image
```

## Configuration

### Target Parameters

Target-tracking parameters are stored in:

```text
drone_control/config/target_params.yaml
```

Example:

```yaml
tracking:
  engagement_distance: 10.0
  attack_distance: 2.0
  max_speed: 5.0
  attack_speed: 10.0

detection:
  confidence_threshold: 0.5
  vehicle_classes: [2, 3, 5, 6, 7]
```

The configured COCO class IDs correspond to:

| Class ID | Object |
| ---: | --- |
| `2` | Car |
| `3` | Motorcycle |
| `5` | Bus |
| `6` | Train |
| `7` | Truck |

### Docker Compose

Useful Docker Compose settings are defined in:

```text
docker-compose.yml
```

Common environment variables:

| Variable | Purpose |
| --- | --- |
| `DISPLAY` | Virtual X display used by the GUI simulation. |
| `ROS_MASTER_URI` | ROS master address. |

Common volume mounts:

| Host Path | Container Path | Purpose |
| --- | --- | --- |
| `./PX4-Autopilot` | `/root/PX4-Autopilot` | PX4 source tree. |
| `./drone_control` | `/root/catkin_ws/src/drone_control` | ROS control package. |

## Troubleshooting

### noVNC Shows the Ubuntu Logo Instead of Gazebo

Enter the container:

```bash
docker exec -it kamikaze_drone bash
```

Restart the display stack:

```bash
pkill -f Xvfb
pkill -f x11vnc
pkill -f websockify
sleep 2

Xvfb :1 -screen 0 1280x800x24 &
sleep 2
DISPLAY=:1 fluxbox &
x11vnc -display :1 -forever -nopw -quiet &
websockify --web=/usr/share/novnc 8080 localhost:5900 &
```

Open:

```text
http://localhost:8080/vnc.html
```

### Drone Is Not Moving

Check MAVROS state:

```bash
rostopic echo /mavros/state
```

Verify that MAVROS is connected:

```text
connected: True
```

Verify that the flight controller is in the expected simulation mode:

```text
mode: "OFFBOARD"
```

### No Target Detection

Check whether the detection topic is publishing:

```bash
rostopic hz /detected_objects
```

If there is no output, run the test detector:

```bash
rosrun drone_control test_detector.py
```

Inspect detections:

```bash
rostopic echo /detected_objects
```

### MAVROS Is Not Connecting

Check MAVROS state:

```bash
rostopic echo /mavros/state
```

If MAVROS is not connected, restart it:

```bash
roslaunch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

### Useful Docker Commands

Follow logs:

```bash
docker logs -f kamikaze_drone
```

Check simulation processes:

```bash
docker exec kamikaze_drone ps aux | grep -E 'px4|mavros|gz|ros'
```

Restart the container:

```bash
docker restart kamikaze_drone
```

Rebuild after code changes:

```bash
docker-compose up -d --build
```

## Project Structure

```text
KamiKaze_Drone/
|-- Dockerfile
|-- LICENSE
|-- README.md
|-- docker-compose.yml
|-- entrypoint.sh
|-- requirements.txt
|-- catkin_ws/
|-- drone_control/
|   |-- CMakeLists.txt
|   |-- package.xml
|   |-- config/
|   |   `-- target_params.yaml
|   |-- launch/
|   |   `-- kamikaze.launch
|   |-- msg/
|   |   |-- BBox.msg
|   |   |-- DetectedObjects.msg
|   |   |-- Object.msg
|   |   `-- Target.msg
|   `-- scripts/
|       |-- camera_calibration.py
|       |-- sensor_fusion.py
|       |-- target_tracking_controller.py
|       |-- test_detector.py
|       |-- vehicle_state_monitor.py
|       `-- yolo_detector.py
`-- PX4-Autopilot/
    `-- PX4 flight stack
```

## Mission States

The target-tracking controller uses the following simulation state machine:

```text
SEARCHING
    |
    | target detected
    v
TRACKING
    |
    | target confirmed
    v
ENGAGING
    |
    | final approach condition reached
    v
ATTACK
```

| State | Description |
| --- | --- |
| `SEARCHING` | Searches for detectable objects in the simulated scene. |
| `TRACKING` | Maintains a track on a detected object. |
| `ENGAGING` | Moves toward the selected simulated target. |
| `ATTACK` | Executes the configured final approach behavior in simulation. |

## Contributing

Contributions are welcome.

1. Fork the repository.
2. Create a feature branch.
3. Make your changes.
4. Test the changes in simulation.
5. Submit a pull request.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for the
full license text.

## Disclaimer

This software is provided for educational and research purposes in simulated
environments. The authors and contributors are not responsible for misuse,
damage, injury, legal consequences, or any other outcomes resulting from the use
or modification of this software.
