FROM ros2_docker:base AS forg_bot_base
FROM ros2_docker:dev AS forg_bot_dev

USER root

#Install forg_bot dependencies
# RUN apt-get update \
    # && apt-get install -y \
    # ros-humble-ament-cmake-ros \
    # ros-humble-ament-lint-auto \
    # ros-humble-ament-lint-common \
    # ros-humble-controller-interface \
    # ros-humble-generate-parameter-library \
    # ros-humble-geometry-msgs \
    # ros-humble-hardware-interface \
    # ros-humble-joint-state-publisher-gui \
    # ros-humble-pluginlib \
    # ros-humble-rclcpp \
    # ros-humble-rclcpp-lifecycle \
    # ros-humble-realtime-tools \
    # ros-humble-sensor-msgs \
    # ros-humble-tf2-geometry-msgs \
    # ros-humble-tf2-ros \
    # ros-humble-xacro \
    # ros-humble-control-msgs \
    # ros-humble-joint-state-broadcaster \
    # ros-humble-gazebo-ros \
    # ros-humble-gazebo-ros2-control \
    # ros-humble-ros2-control \
    # ros-humble-controller-manager \
    # ros-humble-ros2-controllers \
    # && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/ros
WORKDIR /home/ros/

COPY .bashrc /home/ros/.bashrc_append
COPY .zshrc /home/ros/.zshrc_append

RUN echo "\n" >> .bashrc && cat .bashrc_append >> .bashrc \
    && rm .bashrc_append \
    && echo "\n" >> .zshrc && cat .zshrc_append >> .zshrc \
    && rm .zshrc_append
CMD ["zsh"]
USER ros

