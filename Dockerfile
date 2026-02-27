FROM ros2_docker:dev AS forg_bot

USER root

#Install forg_bot dependencies
RUN apt-get update \
    && apt-get install -y \
    # ros-jazzy-joint-state-broadcaster \
    # ros-jazzy-ros2-controllers \
    # ros-jazzy-sensor-msgs \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-ros \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-control-msgs \
    ros-jazzy-controller-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-generate-parameter-library \
    ros-jazzy-geometry-msgs \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-nav-msgs \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-realtime-tools \
    ros-jazzy-ros2-control \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    gdb \
    libeigen3-dev \
    ros-jazzy-eigen3-cmake-module \
    && rm -rf /var/lib/apt/lists/*

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

