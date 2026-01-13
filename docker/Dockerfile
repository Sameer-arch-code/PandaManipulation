FROM moveit/moveit2:humble-humble-tutorial-source

RUN apt update && \
    apt install -y ros-humble-turtlesim && \
    apt install ros-humble-rmw-cyclonedds-cpp -y && \
    echo "source /root/ws_moveit/install/setup.bash" >> /etc/bash.bashrc && \
    rm -rf /var/lib/apt/lists/*

CMD ["tail", "-f", "/dev/null"]
