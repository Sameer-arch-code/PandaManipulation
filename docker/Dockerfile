FROM moveit/moveit2:humble-humble-tutorial-source

RUN apt update && \
    apt install -y ros-humble-turtlesim && \
    apt install ros-humble-example-interfaces && \
    apt install ros-humble-rmw-cyclonedds-cpp -y && \
    apt install ros-humble-imu-filter-madgwick && \
    echo "source /root/ws_moveit/install/setup.bash" >> /etc/bash.bashrc && \
    rm -rf /var/lib/apt/lists/*

CMD ["tail", "-f", "/dev/null"]
