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
    rviz_config = LaunchConfiguration("rviz_config")

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(forg_description / "launch" / "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        remappings=[("joint_states", "input_joint_states")],
    )

    holonomic_rover_kinematics_node = Node(
        package="forg_simulation",
        executable="holonomic_rover_kinematics",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {"debug": True},
            {"front_wheel": "left_front_wheel"},
            {"front_steering_wheel": "left_front_wheel_steering_gear_joint"},
            {"middle_wheel": "left_middle_wheel"},
            {"middle_steering_wheel": "left_middle_wheel_steering_gear_joint"},
            {"rear_wheel": "left_back_wheel"},
            {"rear_steering_wheel": "left_back_wheel_steering_gear_joint"},
            {"front_to_middle_wheel_distance": 0.44},
            {"middle_to_base_link_distance": 0.015},
            {"middle_to_back_wheel_distance": 0.43},
            {"wheel_track": 0.54},
            # {"wheel_track": 1.08},
            # {"twist_input_topic_name": "command_vel"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(forg_simulation / "rviz" / "display.rviz"),
                description="Path to rviz config file",
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="true",
                description="Run robot state publisher if true",
            ),
            robot_state_publisher_launch,
            joint_state_publisher_node,
            holonomic_rover_kinematics_node,
            rviz_node,
        ]
    )
