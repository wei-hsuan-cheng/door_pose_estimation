from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="door_pose_estimation",
                executable="door_pose_estimation_action_server",
                name="door_pose_estimation_action_server",
                output="screen",
                parameters=[
                    {
                        "action_name": "estimate_door_poses",
                        "door_handle_frame_id": "door_handle",
                        "door_hinge_frame_id": "door_hinge",
                        "broadcast_result_tf": True,
                        "door_handle_offset.translation.x": 0.0,
                        "door_handle_offset.translation.y": -0.35,
                        "door_handle_offset.translation.z": 0.0,
                        "door_handle_offset.rotation.roll": 0.0,
                        "door_handle_offset.rotation.pitch": 0.0,
                        "door_handle_offset.rotation.yaw": 0.0,
                        "door_hinge_offset.translation.x": 0.0,
                        "door_hinge_offset.translation.y": 0.45,
                        "door_hinge_offset.translation.z": 0.0,
                        "door_hinge_offset.rotation.roll": 0.0,
                        "door_hinge_offset.rotation.pitch": 0.0,
                        "door_hinge_offset.rotation.yaw": 0.0,
                    }
                ],
            )
        ]
    )
