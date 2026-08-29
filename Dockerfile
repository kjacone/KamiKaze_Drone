# syntax=docker/dockerfile:1.4

# ============================================================
# Application Build Stage
# ============================================================
FROM localhost:5000/kamikaze-drone-base:latest AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ------------------------------------------------------------
# Install build dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-catkin-tools \
    python3-pip \
    wget \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Install Python dependencies
# IMPORTANT:
#   cpuinfo -> WRONG
#   py-cpuinfo -> CORRECT
# ------------------------------------------------------------
RUN pip3 install --no-cache-dir \
    psutil \
    py-cpuinfo

# ------------------------------------------------------------
# Initialize rosdep
# ------------------------------------------------------------
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# ------------------------------------------------------------
# Create catkin workspace
# ------------------------------------------------------------
RUN mkdir -p /root/catkin_ws/src

# ------------------------------------------------------------
# Copy drone_control package
# ------------------------------------------------------------
COPY drone_control/ /root/catkin_ws/src/drone_control/

# ------------------------------------------------------------
# Make Python scripts executable
# ------------------------------------------------------------
RUN if [ -d /root/catkin_ws/src/drone_control/scripts ]; then \
        find /root/catkin_ws/src/drone_control/scripts \
        -type f -name "*.py" -exec chmod +x {} \; ; \
    fi && \
    find /root/catkin_ws/src/drone_control \
        -type f -name "*.py" -exec chmod +x {} \;

# ------------------------------------------------------------
# Install ROS package dependencies
# ------------------------------------------------------------
WORKDIR /root/catkin_ws

RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro noetic \
        -r -y \
    "

# ------------------------------------------------------------
# Build catkin workspace
# ------------------------------------------------------------
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make -j$(nproc) \
    "


# ============================================================
# Runtime Stage
# ============================================================
FROM localhost:5000/kamikaze-drone-base:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ------------------------------------------------------------
# Install runtime dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    python3-pip \
    wget \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Python runtime dependencies
# ------------------------------------------------------------
RUN pip3 install --no-cache-dir \
    psutil \
    py-cpuinfo

# ------------------------------------------------------------
# Install MAVROS GeographicLib datasets
# ------------------------------------------------------------
RUN wget -O /tmp/install_geographiclib_datasets.sh \
    https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x /tmp/install_geographiclib_datasets.sh \
    && /tmp/install_geographiclib_datasets.sh \
    && rm -f /tmp/install_geographiclib_datasets.sh

# ------------------------------------------------------------
# Copy compiled catkin workspace
# ------------------------------------------------------------
COPY --from=builder \
    /root/catkin_ws \
    /root/catkin_ws

# ------------------------------------------------------------
# Create PX4 directory
# ------------------------------------------------------------
RUN mkdir -p /PX4-Autopilot

# ------------------------------------------------------------
# Copy YOLO model if you have one
# Uncomment if yolov8s.pt exists in build context
# ------------------------------------------------------------
COPY data/models/yolov8s.pt /PX4-Autopilot/yolov8s.pt

# Verify model exists inside image
RUN ls -lh /PX4-Autopilot/yolov8s.pt

# ------------------------------------------------------------
# Copy configuration
# ------------------------------------------------------------
COPY drone_control/config /app/config

# ------------------------------------------------------------
# Configure ROS environment
# ------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# ------------------------------------------------------------
# Copy entrypoint
# ------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

# ------------------------------------------------------------
# Runtime environment
# ------------------------------------------------------------
WORKDIR /root/catkin_ws

ENTRYPOINT ["/entrypoint.sh"]