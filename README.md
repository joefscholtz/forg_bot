# Forg Bot

Navigation with [Curiosity Rover](https://science.nasa.gov/mission/msl-curiosity/) inspired holonomic mobile robot in ROS 2. Tested with ROS Jazzy and Gazebo Harmonic on Ubuntu 24.04. This is a meta-package containing the following ROS 2 packages:

- `forg_description`: 3D models and URDFs for the Forg Bot, including simple launchs for visualization with rviz;
- `forg_simulation`: Simulation for the Forg Bot with Gazebo;
- `forg_navigation`: Integration with the Nav2 stack;
- `holonomic_rover_controller`: Controller implemented with the ros2_control framework to control the steering and traction of the 6 Forg Bot's wheels;
- `msg_utils`: General purpose nodes for dealing with `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `nav_msgs/msg/Odometry` and `tf2_msgs/msg/TFMessage` messages.

For more information about each package, take a look in the corresponding `README.md` file inside the package.

## Preview

Forg Bot demo in Gazebo Sim and ICP allocation visualization inside rviz2:

![forg_bot demo](./assets/forg_bot_demo.gif)

## Install

First set up a ROS 2 workspace to use the project and clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:joefscholtz/forg_bot.git
```

### Local

If it is the first time using `rosdep`, run

```bash
sudo rosdep init
rosdep update
```

Install the dependencies with

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --symlink-install
. install/local_setup.bash # or local_setup.zsh if zsh is the shell being used
```

### Using Docker

The provided Dockerfile depends on building the `ros2_docker`image from my [ros2_docker repository](https://github.com/joefscholtz/ros2_docker) which is integrated as a git submodule. Optionally you can use the provided `just` recipes.

First build the images:

```bash
cd ~/ros2_ws/src/forg_bot/ros2_docker
docker compose -f docker-compose.local.yml build
cd ~/ros2_ws/src/forg_bot/
docker compose -f docker-compose.local.yml build
```

Run the container

```bash
CONTAINER_NAME=forg_bot
DOCKER_COMPOSE_SERVICE=forg_bot_app

docker compose -f docker-compose.local.yml run -it --rm --name $CONTAINER_NAME $DOCKER_COMPOSE_SERVICE zsh
```

And build the project

```bash
cd /workspace
colcon build --symlink-install
rc
```

You can enter the container in other terminals with

```bash
CONTAINER_NAME=forg_bot

docker exec -it $CONTAINER_NAME zsh
```

To launch the kinematics visualization run inside the container

```bash
ros2 launch forg_simulation display_kinematics.launch.py
```

To launch the simulation in Gazebo run inside the container

```bash
ros2 launch forg_navigation navigation.launch.py
```

## TODO

- [ ] Add license;
- [ ] Maybe it is a good idea to use FreeCAD + ROS 2 integration in the future. Sources:
  - [FreeCAD ROS Workbench](https://github.com/galou/freecad.cross);
  - [FreeCAD CAD & ROS Open-Source Synergy (CROSS)](https://github.com/drfenixion/freecad.overcross);
  - [Freecad to Gazebo Exporter](https://github.com/Dave-Elec/freecad_to_gazebo).
- [ ] Set up Ignition;
- [ ] Publish `base_footprint` frame as being the projection of the `base_link` frame in the ground (I think it is best to do it in the `forg_navigation` package).

## Special thanks

- The robot design was heavily inspired by [WildWilly's Stair Climbing Rover](https://www.printables.com/model/194299-stair-climbing-rover)
