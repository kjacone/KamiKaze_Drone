FROM ros:noetic-ros-base

# Combine all apt-get installations into one layer
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
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip to avoid hash mismatches
RUN pip3 install --upgrade pip

# Install PyTorch CPU-only from the official PyTorch index (smaller ~200MB)
RUN pip3 install --no-cache-dir --default-timeout=1000 --retries=5 \
    --index-url https://download.pytorch.org/whl/cpu \
    torch==2.0.1 \
    torchvision==0.15.2

# Install the rest of the Python packages from the default PyPI
RUN pip3 install --no-cache-dir --default-timeout=1000 --retries=5 \
    ultralytics \
    filterpy \
    scipy \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    toml \
    packaging \
    future

# Install geographiclib dataset (warn but don't fail if it times out)
RUN for i in 1 2 3 4 5; do \
        geographiclib-get-geoids egm96-5 && exit 0; \
        echo "geographiclib-get-geoids retry $i..."; sleep 5; \
    done; \
    echo "geographiclib-get-geoids failed, trying mavros installer..." && \
    bash /opt/ros/noetic/lib/mavros/install_geographiclib_dataset.sh || \
    echo "WARNING: GeographicLib dataset not installed – some features may not work"

# Create catkin workspace directory
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root