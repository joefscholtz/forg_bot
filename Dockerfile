FROM ros2_docker AS forg_bot

USER root

#Install forg_bot dependencies
RUN apt-get update \
  && apt-get install -y \
  ros-jazzy-rclcpp \
  ros-jazzy-rclcpp-lifecycle \
  ros-jazzy-pluginlib \
  ros-jazzy-xacro \
  ros-jazzy-realtime-tools \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-controller-interface \
  ros-jazzy-hardware-interface \
  ros-jazzy-hardware-interface-testing \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-control-msgs \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ament-cmake \
  ros-jazzy-ament-cmake-ros \
  ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-lint-common \
  ros-jazzy-generate-parameter-library \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-navigation2 \
  libeigen3-dev \
  libmodbus-dev \
  && rm -rf /var/lib/apt/lists/*

RUN git config --global --add safe.directory /workspace

COPY .zshrc /home/ros/.zshrc_append

RUN echo "\n" >> .zshrc && \cat .zshrc_append >> .zshrc \
    && rm .zshrc_append

CMD ["zsh"]
USER ros
