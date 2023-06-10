FROM dustynv/ros:foxy-ros-base-l4t-r32.5.0

RUN apt-get update && \
    apt-get install --no-install-recommends -y libudev-dev && \
    apt-get clean && apt-get autoclean && apt-get autoremove && \ 
    rm -rf /app/wheels && \
    mkdir -p workspace/ros2_ws/src

COPY unitree_cam_publisher workspace/ros2_ws/src

RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd workspace/ros2_ws && \
    colcon build

RUN echo "ALL DONE!"