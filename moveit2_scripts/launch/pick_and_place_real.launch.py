import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
            {
            "object_pose.frame_id": "base_link",
            "object_pose.x": 0.360,
            "object_pose.y": 0.132,
            "object_pose.z": 0.05,
        }],
        remappings=[("/joint_states", "/rrl/actual_joint_states")],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )