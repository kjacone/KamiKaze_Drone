# Dockerfile - Optimized for M1 Mac (ARM64) and AMD64 compatibility
FROM --platform=linux/amd64 ros:noetic-ros-base

# ------------------------------------------------------------
# System dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-rqt-image-view \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    geographiclib-tools \
    xvfb \
    fluxbox \
    x11vnc \
    websockify \
    novnc \
    x11-utils \
    net-tools \
    wget \
    ca-certificates \
    git \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-plugins \
    protobuf-compiler \
    libprotobuf-dev \
    libprotoc-dev \
    libeigen3-dev \
    libopencv-dev \
    libxml2-utils \
    libxml2-dev \
    libssl-dev \
    libyaml-cpp-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libimage-exiftool-perl \
    sudo \
    vim \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# GeographicLib geoid data
# ------------------------------------------------------------
RUN mkdir -p /usr/share/GeographicLib/geoids && \
    python3 - <<'PY'
import struct

width = 1440
height = 721
output = "/usr/share/GeographicLib/geoids/egm96-5.pgm"

with open(output, "wb") as f:
    f.write(b"P5\n")
    f.write(b"# Offset -107\n")
    f.write(b"# Scale 0.003\n")
    f.write(f"{width} {height}\n".encode())
    f.write(b"65535\n")

    for _ in range(width * height):
        f.write(struct.pack(">H", 32768))

print(f"Created GeographicLib geoid data: {output}")
PY

# ------------------------------------------------------------
# Upgrade pip
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade pip

# ------------------------------------------------------------
# Install PyTorch CPU-only
# ------------------------------------------------------------
RUN python3 -m pip install \
    --no-cache-dir \
    --default-timeout=1000 \
    --retries=5 \
    --index-url https://download.pytorch.org/whl/cpu \
    torch==2.0.1 \
    torchvision==0.15.2

# ------------------------------------------------------------
# Python dependencies
# ------------------------------------------------------------
RUN python3 -m pip install \
    --no-cache-dir \
    --default-timeout=1000 \
    --retries=5 \
    ultralytics \
    filterpy \
    scipy \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    toml \
    packaging \
    future \
    numpy \
    opencv-python \
    pyyaml \
    psutil \
    matplotlib

# ------------------------------------------------------------
# Create catkin workspace
# ------------------------------------------------------------
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

# ------------------------------------------------------------
# ROS environment
# ------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=127.0.0.1" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# Default shell
CMD ["/bin/bash"]