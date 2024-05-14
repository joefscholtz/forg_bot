# Forg Bot

Navigation with [Curiosity Rover](https://science.nasa.gov/mission/msl-curiosity/) inspired holonomic mobile robot in ROS 2. Tested with ROS Humble on Ubuntu 22.04 Jammy Jellyfish. This is a meta-package containing the following ROS 2 packages:

- `forg_description`: 3D models and URDFs for the Forg Bot, including simple launchs for visualization with rviz;
- `holonomic_rover_controller`: Controller implemented with the ros2_control framework to control the steering and traction of the 6 Forg Bot's wheels;
- `msg_utils`: General purpose nodes for dealing with `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `nav_msgs/msg/Odometry` and `tf2_msgs/msg/TFMessage` messages.

For more information about each package, take a look in the corresponding `README.md` file inside the package.

## Preview

Forg Bot model displayed in rviz2:

![Rviz2 display](assets/forg_description-display.png)

## Install

First set up a ROS 2 workspace to use the project and clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:joefscholtz/forg_bot.git
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

The provided Dockerfile depends on building the `ros2_docker:base` or `ros2_docker:dev` image from my [ros2_docker repository](https://github.com/joefscholtz/ros2_docker), so follow the instrutions from the ros2_docker README first. Having one of the ros2_docker images, the forg_bot images can be build with

```bash
cd ~/ros2_ws/src/forg_bot/
docker build --target forg_bot_dev --tag forg_bot:dev . #or base
```

It is also possible to build with the provided script with

```bash
cd ~/ros2_ws/src/forg_bot/
chmod +x docker_build.sh
./docker_build.sh
```

And then run with

```bash
cd ~/ros2_ws/src/forg_bot/
./x11run.sh
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
