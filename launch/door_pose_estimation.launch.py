from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("door_pose_estimation"), "config", "door_params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="door_pose_estimation",
                executable="door_pose_estimation_action_server",
                name="door_pose_estimation_action_server",
                output="screen",
                parameters=[
                    params_file
                ],
            )
        ]
    )
