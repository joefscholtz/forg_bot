FROM ros2_docker:base AS forg_bot:base
FROM ros2_docker:dev AS forg_bot:dev

USER root

#Install outside-bringup dependencies
RUN apt-get update \
    # && apt-get install -y \
    # ros-humble-control-msgs \
    # ros-humble-joint-state-broadcaster \
    # ros-humble-gazebo-ros \
    # ros-humble-gazebo-ros2-control \
    # ros-humble-ros2-control \
    # ros-humble-controller-manager \
    # ros-humble-ros2-controllers \
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

