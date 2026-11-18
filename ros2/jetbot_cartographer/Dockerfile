FROM ros:humble-ros-base

# Install Google Cartographer and ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir websockets numpy

WORKDIR /ros2_ws
COPY . /ros2_ws/src/jetbot_cartographer

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select jetbot_cartographer

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch jetbot_cartographer cartographer.launch.py ws_url:=$0 \"$@\""]
CMD ["ws://127.0.0.1:8000/ws/robot"]
