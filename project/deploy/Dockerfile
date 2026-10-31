FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz ros-humble-tf2-ros ros-humble-nav2-msgs python3 mesa-utils libegl1 libgl1 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY . /workspace
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
ENV IGN_GAZEBO_RESOURCE_PATH=/workspace/assets/models
CMD ["bash", "/workspace/deploy/run.sh", "server"]
