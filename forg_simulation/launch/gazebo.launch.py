from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    forg_description = get_package_share_path("forg_description")
    forg_simulation = get_package_share_path("forg_simulation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_bridge_params = LaunchConfiguration("gz_bridge_params")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    gazebo_verbose = LaunchConfiguration("gazebo_verbose")
    gazebo_debug = LaunchConfiguration("gazebo_debug")
    world = LaunchConfiguration("world")

    pose = {
        "x": LaunchConfiguration("x_pose"),
        "y": LaunchConfiguration("y_pose"),
        "z": LaunchConfiguration("z_pose"),
        "R": LaunchConfiguration("roll"),
        "P": LaunchConfiguration("pitch"),
        "Y": LaunchConfiguration("yaw"),
    }

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(forg_description / "launch" / "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(get_package_share_path("ros_gz_sim") / "launch" / "gz_sim.launch.py")]
        ),
        launch_arguments={
            # https://github.com/ros-controls/gz_ros2_control/issues/340
            "gz_args": ["-r -v4 ", world, " --physics-engine gz-physics-bullet-featherstone-plugin"],
            # + str(forg_description / "config" / "gazebo.yaml")
            "on_exit_shutdown": "true",
            "use_sim_time": use_sim_time,
            "verbose": gazebo_verbose,
            "debug": gazebo_debug,
        }.items(),
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="forg_bot_spawner",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "forg_bot",
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["holonomic_rover_controller"],
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(get_package_share_path("ros_gz_bridge") / "launch" / "ros_gz_bridge.launch.py")]
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
            "config_file": gz_bridge_params,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="true",
                description="Run robot state publisher if true",
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="false",
                description="Gazebo verbose tag, boolean",
            ),
            DeclareLaunchArgument(
                "gazebo_debug",
                default_value="false",
                description="Gazebo debug tag, boolean",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(forg_simulation / "worlds" / "empty.world"),
                description="Which world to launch in Gazebo",
            ),
            DeclareLaunchArgument(
                "gz_bridge_params",
                default_value=str(forg_simulation / "config" / "gz_bridge.yaml"),
                description="ros_gz_bridge parameters file path as string",
            ),
            DeclareLaunchArgument(
                "x_pose",
                default_value="0.0",
                description="Robot's x coordinate to spawn",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="0.0",
                description="Robot's y coordinate to spawn",
            ),
            DeclareLaunchArgument(
                "z_pose",
                default_value="0.09",
                description="Robot's z coordinate to spawn",
            ),
            DeclareLaunchArgument(
                "roll",
                default_value="0.0",
                description="Robot's roll angle to spawn",
            ),
            DeclareLaunchArgument(
                "pitch",
                default_value="0.0",
                description="Robot's pitch angle to spawn",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Robot's yaw angle to spawn",
            ),
            robot_state_publisher_launch,
            gazebo_launch,
            spawn_entity_node,
            controller_spawner,
            ros_gz_bridge,
        ]
    )
