FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-foxglove-msgs \
        python3-serial \
        python3-gi \
        python3-colcon-common-extensions \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gir1.2-gstreamer-1.0 \
        gir1.2-gst-plugins-base-1.0 \
        git \
    && rm -rf /var/lib/apt/lists/*
ENV CMAKE_PREFIX_PATH=/opt/ros/jazzy
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export PYTHONPATH=/root/ros_ws:\$PYTHONPATH" >> /root/.bashrc && \
    echo "[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /root/ros_ws
