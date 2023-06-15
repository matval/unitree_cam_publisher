FROM dustynv/ros:foxy-ros-base-l4t-r32.5.0

RUN apt-get update && \
    apt-get install --no-install-recommends -y libudev-dev && \
    apt-get remove -y thunderbird libreoffice-* gazebo9* && \
    apt-get clean && apt-get autoclean && apt-get autoremove -y && \ 
    rm -rf /app/wheels && rm -rf /var/lib/apt/lists/* && \
    mkdir -p /root/ros2_ws/src

COPY unitree_cam_publisher /root/ros2_ws/src

RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd /root/ros2_ws && \
    colcon build && \
    echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

# setup container entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh

RUN echo "ALL DONE!"
