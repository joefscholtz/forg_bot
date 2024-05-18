from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# import os.path as osp


def generate_launch_description():

    # pkg_path = osp.join(get_package_share_directory("pkg_name"))
    forg_description = get_package_share_path("forg_description")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # xacro_file = osp.join(pkg_path, "description", "robot.urdf.xacro")
    xacro_file = forg_description / "urdf" / "forg_bot.urdf.xacro"
    robot_description_config = ParameterValue(
        Command(
            [
                "xacro ",
                str(xacro_file),
                " simulation:=",
                use_sim_time,
            ],
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            robot_state_publisher_node,
        ]
    )
