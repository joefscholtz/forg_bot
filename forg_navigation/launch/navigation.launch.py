from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    forg_simulation = get_package_share_path("forg_simulation")
    forg_navigation = get_package_share_path("forg_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration('log_level')
    nav2_params = LaunchConfiguration('nav2_params')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(forg_simulation / "launch" / "gazebo.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    lifecycle_nodes = [
        'velocity_smoother',
    ]
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
        + [('cmd_vel', 'cmd_vel_nav'),('cmd_vel_smoothed', 'holonomic_rover_controller/reference')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                'log_level', default_value='info', description='log level'
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(forg_navigation / "config" / "nav2_params.yaml"),
                description="nav2 parameters file path as string",
            ),
            gazebo_launch,
            nav2_lifecycle_manager,
            velocity_smoother,
        ]
    )
