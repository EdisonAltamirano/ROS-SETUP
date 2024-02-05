FROM osrf/ros:noetic-desktop-full

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano\
    python3-pip -y \
    gedit \
    --no-install-recommends terminator \
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

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /home/ws/devel/setup.bash" >> /root/.bashrc