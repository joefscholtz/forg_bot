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
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    gazebo_verbose = LaunchConfiguration("gazebo_verbose")
    gazebo_debug = LaunchConfiguration("gazebo_debug")
    world = LaunchConfiguration("gazebo_debug")

    pose = {
        "x": LaunchConfiguration("x_pose", default="0.0"),
        "y": LaunchConfiguration("y_pose", default="0.0"),
        "z": LaunchConfiguration("z_pose", default="0.0"),
        "R": LaunchConfiguration("roll", default="0.0"),
        "P": LaunchConfiguration("pitch", default="0.0"),
        "Y": LaunchConfiguration("yaw", default="0.0"),
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
            [str(get_package_share_path("gazebo_ros") / "launch" / "gazebo.launch.py")]
        ),
        launch_arguments={
            # "extra_gazebo_args": "--ros-args --params-file "
            # + str(forg_description / "config" / "gazebo.yaml")
            "pause": "true",
            "params_file": str(forg_simulation / "config" / "gazebo.yaml"),
            "use_sim_time": use_sim_time,
            "verbose": gazebo_verbose,
            "debug": gazebo_debug,
            "world": world,
        }.items(),
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="forg_bot_spawner",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
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
                default_value="worlds/empty_sky.world",
                description="Which world to launch in Gazebo",
            ),
            robot_state_publisher_launch,
            gazebo_launch,
            spawn_entity_node,
        ]
    )
