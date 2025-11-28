FROM ros:humble-ros-base

# Çalışma alanı
WORKDIR /ws

# Gerekli araçlar
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Proje kaynaklarını kopyala
COPY . /ws

# entrypoint için çalıştırma izni
RUN chmod +x /ws/entrypoint.sh

# Colcon build
RUN . /opt/ros/humble/setup.sh && \
    colcon build

ENV ROS_DISTRO=humble

ENTRYPOINT ["/ws/entrypoint.sh"]


