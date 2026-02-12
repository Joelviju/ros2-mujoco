from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---------------------------
    # MoveIt (CHOMP-only)
    # ---------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_moveit_mujoco_package"),
                "launch",
                "moveit_only.launch.py"
            )
        )
    )

    # ---------------------------
    # MuJoCo + FollowJointTrajectory server
    # ---------------------------
    mujoco_node = Node(
        package="mujoco_ros2_bridge",
        executable="mujoco_follow_joint_trajectory",
        name="mujoco_follow_joint_trajectory",
        output="screen",
        arguments=[
            "/workspaces/ros2-mujoco/mujoco/menagerie/universal_robots_ur10e/scene.xml"
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ---------------------------
    # MoveIt client (your C++ node)
    # ---------------------------
    moveit_client_node = Node(
        package="mujoco_ros2_bridge",
        executable="ur10e_moveit_robot",
        name="ur10e_moveit_robot",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        mujoco_node,
        moveit_launch,
        moveit_client_node,
    ])
