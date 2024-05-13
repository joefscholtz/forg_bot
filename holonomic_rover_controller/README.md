# Holonomic Rover Controller

Traction and steering ros2_control controller for the kinematics of a rover with 6 orientable and tractionable wheels.

## Sources:

- [ros2_control "Writing a new controller" page](https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html);
- [ros2_control Github](https://github.com/ros-controls/ros2_control), in particular `controller_interface` and `chainable_controller_interface`inside the `controller_interface` package;
- [ros2_controllers Github](https://github.com/ros-controls/ros2_controllers), in particular the `steering_controllers_library` package;
- [PickNikRobotics generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library).

## TODO:

- [] Implement the controller;
- [] Implement odometry;
- [] Add testing.
