FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-yaml \
        ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2 \
        ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY tools/judge_run.sh tools/eval_recorded.py /tools/
COPY docker/fastdds_udp_only.xml /etc/fastdds_udp_only.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp_only.xml
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
