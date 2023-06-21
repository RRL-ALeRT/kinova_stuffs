import os

import launch
import launch_ros
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> launch.LaunchDescription:
    default_model_path = os.path.join(get_package_share_directory("kortex_description"), "robots/gen3_robotiq_2f_140.xacro")

    controller = Node(
        package='kortex_controller_py',
        executable='command',
        output='screen',
    )

    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            ["xacro ", default_model_path, " dof:=6"]
                        )
                    }
                ],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    kinova_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kinova_vision'), 'launch', 'rgbd_launch.py')),
    )

    kinova_pointcloud2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kinova_vision'), 'launch', 'point_cloud_xyzrgb_launch.py')),
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", default_model_path, " dof:=6"]
                )
            }
        ],
    )

    rviz_config_file = 'src/kinova_stuffs/kortex_controller_py/rivz/spot.rviz'

    # RViz2 node
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )


    return launch.LaunchDescription(
        [
            controller,
            kinova_vision,
            kinova_pointcloud2,
            robot_state_publisher_node,
            container,
            rviz_node
        ]
    )