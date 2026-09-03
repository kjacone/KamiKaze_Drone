# 🚁 Autonomous Kamikaze Drone Project

> **Educational and research use only.**

This project is intended for controlled simulation environments. It is **not recommended for real-world surveillance, tracking or operational deployment**. Users are responsible for complying with all applicable laws, safety requirements, and organizational policies.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
  - [Core Components](#core-components)
  - [Data Flow](#data-flow)
  - [Monitoring](#monitoring)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Docker Setup](#docker-setup)
  - [Native Ubuntu Setup](#native-ubuntu-setup)
- [Running the Project](#running-the-project)
  - [Docker Mode](#docker-mode)
  - [Native Mode](#native-mode)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Mission and Control Components](#mission-and-control-components)
- [Network Ports](#network-ports)
- [Development](#development)
- [Contributing](#contributing)
- [License](#license)
- [Disclaimer](#disclaimer)

---

## Overview

This repository contains a ROS-based autonomous drone simulation combining:

| Technology | Purpose |
|---|---|
| **YOLOv8** | Simulated object detection |
| **ROS Noetic** | Robotics middleware |
| **MAVROS** | ROS ↔ MAVLink/PX4 communication |
| **PX4 SITL** | Simulated flight controller |
| **Gazebo Classic** | 3D simulation environment |
| **Docker** | Reproducible development environment |
| **noVNC** | Browser-based graphical simulation access |
| **Prometheus** | Metrics collection |
| **Grafana** | Monitoring and visualization |
| **Node Exporter** | Host/system metrics |
| **cAdvisor** | Container metrics |

The project is structured for experimenting with simulated perception, vehicle-state monitoring, target tracking, trajectory planning, control loops, safety monitoring, and system observability.

All flight behavior described in this repository is intended to remain inside the simulation environment.

---

# System Architecture

## Core Components

The main Docker Compose stack is organized around the `kamikaze_drone` container and an optional monitoring stack.

```text
┌──────────────────────────────────────────────────────────────────────┐
│                         DOCKER COMPOSE STACK                         │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────────┐       ┌──────────────────────────────┐    │
│  │   kamikaze_drone     │       │      Monitoring Stack        │    │
│  │   Main Container     │──────▶│ Prometheus / Grafana / etc. │    │
│  └──────────┬───────────┘       └──────────────────────────────┘    │
│             │                                                        │
│             ▼                                                        │
│  ┌──────────────────────┐                                            │
│  │      ROS Master      │                                            │
│  │       roscore        │                                            │
│  └──────────┬───────────┘                                            │
│             │                                                        │
│             ▼                                                        │
│  ┌──────────────────────┐                                            │
│  │   PX4 SITL + Gazebo  │                                            │
│  └──────────┬───────────┘                                            │
│             │                                                        │
│             ▼                                                        │
│  ┌──────────────────────┐                                            │
│  │       MAVROS         │                                            │
│  │     FCU Bridge       │                                            │
│  └──────────┬───────────┘                                            │
│             │                                                        │
│             ▼                                                        │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    ROS Control Nodes                         │   │
│  │ YOLO → Tracking → Planning → Control → Safety/Monitoring    │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

### Component Connections

```text
Gazebo
   │
   ▼
PX4 SITL
   │
   ▼
MAVROS
   │
   ▼
ROS Control Nodes
   │
   ├── YOLO Detector
   ├── Target Tracker
   ├── Trajectory Planner
   ├── Predictive Controller
   ├── Collision Avoidance
   └── Performance Monitor
```

### Service Dependencies

```text
kamikaze_drone
│
├── ROS Master (roscore)
│
├── Gazebo
│   └── Port 11345
│
├── PX4 SITL
│   ├── TCP 4560
│   ├── UDP 14540
│   └── UDP 14580
│
├── MAVROS
│   └── FCU URL: udpin://:14540@127.0.0.1:14557
│
└── ROS Control Nodes
    ├── YOLO detector
    ├── Trajectory planner
    ├── Predictive controller
    ├── Collision avoidance
    └── Performance monitor

Monitoring
│
├── Prometheus
│   └── Port 9090
│
├── Grafana
│   └── Port 3000
│
├── Node Exporter
│   └── Port 9100
│
└── cAdvisor
    └── Port 8081
```

---

## Data Flow

### Control Loop

```text
Camera Image
     │
     ▼
YOLO Detector
     │
     ▼
Detection Array
     │
     ▼
Target Tracking
     │
     ▼
Trajectory Planner
     │
     ▼
Path
     │
     ▼
Predictive Controller
     │
     ▼
Control Command
     │
     ▼
MAVROS
     │
     ▼
PX4 SITL
     │
     ▼
Gazebo
     │
     ▼
State Feedback
     │
     ▼
Sensor Fusion / EKF
```

### Monitoring Flow

```text
ROS Nodes
    │
    ▼
Performance Monitor
    │
    ▼
Prometheus
    │
    ▼
Grafana
```

Metrics include:

- CPU and memory statistics
- System health
- Control latency
- Throughput
- Custom application metrics
- Container metrics
- Time-series monitoring

---

# Prerequisites

## Docker Setup

The Docker workflow is recommended for a consistent development and simulation environment.

### Minimum

- Docker Engine **20.10+**
- Docker Compose **1.29+** or Compose v2
- **8 GB RAM**
- **20 GB free disk space**

### Recommended

- **16 GB RAM or more**
- Hardware virtualization enabled
- NVIDIA GPU for faster YOLO inference, where supported

## Native Ubuntu Setup

The native workflow requires:

- Ubuntu **20.04 LTS**
- ROS **Noetic**
- PX4 Autopilot
- MAVROS
- MAVROS Extras
- Gazebo Classic
- Python 3
- NVIDIA GPU — optional

> **Note:** ROS Noetic is designed for Ubuntu 20.04. Native installation on other Ubuntu versions may require additional compatibility work.

---

# Installation

## Docker Setup

### 1. Clone the Repository

Clone the repository together with the PX4 submodule:

```bash
git clone --recurse-submodules https://github.com/kjacone/KamiKaze_Drone.git
cd KamiKaze_Drone
```

Start PX4 indepently using Docker

```bash
docker run --rm \                       
  -p 14540:14540/udp \    
  -p 14550:14550/udp \
  --name px4_sitl \         
  jonasvautherin/px4-gazebo-headless:latest \
  127.0.0.1
```



> **Important:** Validate the entire simulation after changing the PX4 revision.

### 4. Build the Docker Image

```bash
docker-compose build
```

### 5. Start the Container

```bash
docker-compose up -d
```

### 6. Verify the Container

```bash
docker ps | grep kamikaze_drone
```

### 7. View Logs

```bash
docker logs kamikaze_drone
```

### 8. Open the Simulation

Install QGroundControl to view the drone in realtime

```text
https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html
```

---

## Native Ubuntu Setup

### 1. Install ROS Noetic

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install MAVROS

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

### 3. Install PX4 Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
cd ..
```

### 4. Install Gazebo ROS Packages

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### 5. Install Python Dependencies

From the project root:

```bash
pip3 install -r requirements.txt
```

### 6. Clone the Project

```bash
cd ~
git clone https://github.com/kjacone/KamiKaze_Drone.git
cd KamiKaze_Drone
```

### 7. Create the ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

ln -s ~/KamiKaze_Drone/drone_control .

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 8. Copy the Modified Iris Model

If the simulation requires the project's modified Iris model:

```bash
cp ~/KamiKaze_Drone/modified_px4/Tools/simulation/gazebo-classic/models/iris/iris.sdf.jinja \
   ~/PX4-Autopilot/Tools/simulation/gazebo-classic/models/iris/
```

---

# Running the Project

## Docker Mode

### 1. Start the Simulation

```bash
docker-compose up -d
```

### 2. Open [QGroundControl](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html)



### 3. Enter the Container

```bash
docker exec -it kamikaze_drone bash
```

### 4. Source the ROS Workspace

```bash
source /root/catkin_ws/devel/setup.bash
```

### 5. Run the Test Detector

```bash
rosrun drone_control test_detector.py
```

### 6. Run the YOLO Detector

```bash
rosrun drone_control yolo_detector.py
```

### 7. Monitor Velocity Commands

```bash
rostopic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

### 8. Stop the Simulation

```bash
docker-compose down
```

---

## Native Mode

The native workflow uses separate terminals for PX4, MAVROS, monitoring, control, and visualization.

### Terminal 1 — Launch PX4 SITL

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

### Terminal 2 — Launch MAVROS

```bash
source ~/catkin_ws/devel/setup.bash

roslaunch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

### Terminal 3 — Start Vehicle State Monitor

```bash
source ~/catkin_ws/devel/setup.bash
rosrun drone_control vehicle_state_monitor.py
```

### Terminal 4 — Start Target Tracking Controller

```bash
source ~/catkin_ws/devel/setup.bash
rosrun drone_control target_tracking_controller.py
```

### Terminal 5 — Visualize Processed Images

Start `rqt`:

```bash
rqt
```

Then open:

**Plugins → Visualization → Image View**

Select:

```text
/processed_image
```

---

# Configuration

Configuration files are located under:

```text
drone_control/config/
```

The repository contains configuration for tracking, mission behavior, flight control, camera calibration, filtering, system parameters, testing, debugging, diagnostics, and test scenarios.

## Target Parameters

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

Configured COCO class IDs:

| Class ID | Object |
|---:|---|
| `2` | Car |
| `3` | Motorcycle |
| `5` | Bus |
| `6` | Train |
| `7` | Truck |

> These values are part of the simulation configuration. Changes should be tested in simulation before being used in other environments.

## Docker Compose

Docker configuration is defined in:

```text
docker-compose.yml
```

### Common Environment Variables

| Variable | Purpose |
|---|---|
| `DISPLAY` | Virtual X display used by the GUI simulation |
| `ROS_MASTER_URI` | ROS master address |

### Common Volume Mounts

| Host Path | Container Path | Purpose |
|---|---|---|
| `./PX4-Autopilot` | `/root/PX4-Autopilot` | PX4 source tree |
| `./drone_control` | `/root/catkin_ws/src/drone_control` | ROS control package |

---

# Troubleshooting


## Drone Is Not Moving

Check MAVROS state:

```bash
rostopic echo /mavros/state
```

Verify that MAVROS reports:

```text
connected: True
```

Also verify that the flight controller is in the expected simulation mode:

```text
mode: "OFFBOARD"
```

Check whether the controller is publishing setpoints:

```bash
rostopic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

---

## No Target Detection

Check whether the detection topic is publishing:

```bash
rostopic hz /detected_objects
```

If there is no output, run the test detector:

```bash
rosrun drone_control test_detector.py
```

Inspect detection messages:

```bash
rostopic echo /detected_objects
```

---

## MAVROS Is Not Connecting

Check MAVROS state:

```bash
rostopic echo /mavros/state
```

Restart MAVROS:

```bash
roslaunch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

---

# Useful Docker Commands

### Follow Logs

```bash
docker logs -f kamikaze_drone
```

### Check Simulation Processes

```bash
docker exec kamikaze_drone ps aux | grep -E 'px4|mavros|gz|ros'
```


### Rebuild from Scratch

```bash
# Start full stack
docker compose --profile full up --build

# Start with monitoring only
docker compose --profile monitoring up --build

# Start predictive only
docker compose --profile predictive up --build

# Stop everything
docker compose --profile full down

# Clean everything including volumes
docker compose --profile full down -v



# Check all containers are running
docker compose --profile full ps

# Check ROS nodes
docker exec kamikaze_drone rosnode list

# Check predictive can connect to ROS
docker exec predictive_controller rosnode list

# Check services are accessible
curl http://localhost:9090  # Prometheus
curl http://localhost:3000  # Grafana
```

### Quick ROS Diagnostics

```bash
# Check whether ROS Master is running
docker exec kamikaze_drone rostopic list

# List running ROS nodes
docker exec kamikaze_drone rosnode list

# Test the YOLO detector
docker exec kamikaze_drone rosrun drone_control yolo_detector.py --help

# Check Prometheus metrics
curl http://localhost:9090/metrics

# Check Grafana health
curl http://localhost:3000/api/health
```

--

# Mission and Control Components

The ROS package is organized into functional areas:

| Area | Components |
|---|---|
| **Detection** | YOLO detector, object tracker |
| **Control** | Flight controller, target tracking, trajectory planning |
| **Navigation** | Waypoint navigator, trajectory planner |
| **Prediction** | Predictive controller |
| **Safety** | Collision avoidance, safety monitor |
| **Mission** | Mission manager, mission status |
| **State** | Vehicle state monitor, sensor fusion |
| **Diagnostics** | Health checker, diagnostic reporter |
| **Utilities** | Parameter validation/loading, error handling |
| **Testing** | Test detector, recorder, verification, simulated target |

---

# Network Ports

| Component | Port | Purpose |
|---|---:|---|
| ROS Master | `11311` | ROS service discovery |
| Gazebo | `11345` | Gazebo communication |
| PX4 SITL | `4560` | Simulator connection |
| PX4 SITL | `14540` | MAVLink communication |
| PX4 SITL | `14580` | MAVLink onboard |
| MAVROS | `14557` | FCU bridge |
| Prometheus | `9090` | Metrics collection |
| Grafana | `3000` | Monitoring dashboard |
| noVNC | `8080` | Browser-based GUI |

---

# Development

## Recommended Development Workflow

1. Make changes inside `drone_control/`.
2. Rebuild the ROS workspace or Docker image.
3. Start the simulation.
4. Verify ROS topics and node health.
5. Test perception and control behavior in simulation.
6. Review Prometheus/Grafana metrics.
7. Run the relevant test scripts.
8. Document configuration or behavioral changes.

## Basic Verification

```bash
# ROS topics
rostopic list

# ROS nodes
rosnode list

# MAVROS state
rostopic echo /mavros/state

# Detection rate
rostopic hz /detected_objects

# Velocity commands
rostopic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

---

# Contributing

Contributions should remain focused on **simulation, research, safety, testing, and educational use**.

Before submitting changes:

- Test changes inside the simulation environment.
- Document new dependencies.
- Document configuration changes.
- Avoid committing secrets or credentials.
- Verify Docker and native workflows where applicable.
- Include relevant diagnostics when changing ROS nodes or communication flows.

---

# License

See [`LICENSE`](LICENSE) for the project's licensing terms.

---

# Disclaimer

> **This repository is provided for controlled simulation, educational, and research purposes.**

The project contains autonomous perception, tracking, planning, and control components. These capabilities should be evaluated only in appropriate simulated or otherwise authorized environments.

**Do not deploy the system for real-world weaponization, harmful targeting, unauthorized surveillance, or other unsafe or unlawful activity.**

Users are responsible for ensuring that their use of this software complies with applicable laws, regulations, safety requirements, and organizational policies.

---

## Documentation Source

This README was organized from the project documentation supplied with this repository. Technical commands, paths, ports, components, and configuration examples were retained from the supplied material.
