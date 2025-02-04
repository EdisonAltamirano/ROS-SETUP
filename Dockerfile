# Specify the platform as linux/amd64 or linux/arm64 based on your system
FROM osrf/ros:humble-desktop

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano \
    python3-pip \
    gedit \
    terminator \
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    espeak \
    alsa-utils \
    software-properties-common \
    ffmpeg \
    bluez \
    portaudio19-dev \
    pulseaudio-module-bluetooth \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup files
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /root/.bashrc
