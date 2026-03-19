import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import launch
import launch_ros
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> launch.LaunchDescription:
    default_model_path = os.path.join(get_package_share_directory("kortex_description"), "robots/gen3.urdf")

    controller = Node(
        package='kortex_controller_py',
        executable='trajectory_server',
        output='screen',
    )

    joint_states = Node(
        package='kortex_controller_py',
        executable='joint_states_publisher',
        output='screen',
        
    )

    # kinova_vision = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('kinova_vision'), 'launch', 'rgbd_launch.py')),
    # )

    # kinova_pointcloud2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('kinova_vision'), 'launch', 'point_cloud_xyzrgb_launch.py')),
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": open(default_model_path, "r").read()
            }
        ],
    )

    return launch.LaunchDescription(
        [
            controller,
            # joint_states,
            # kinova_vision,
            # kinova_pointcloud2,
            robot_state_publisher_node,
        ]
    )
