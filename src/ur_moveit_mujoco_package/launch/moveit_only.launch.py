from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_moveit_mujoco_package")
        .robot_description(file_path="config/ur10e.urdf.xacro")
        .robot_description_kinematics("config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        # 🔑 THIS IS THE CRITICAL PART (controller wiring)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # 🔑 Ensure SRDF + URDF are published
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    #rviz_config_path = os.path.join(
        #moveit_config.package_path, "config", "moveit.rviz"
    #)

    #rviz_node = Node(
        #package="rviz2",
        #executable="rviz2",
        #output="screen",
        #arguments=["-d", rviz_config_path],
        #parameters=[{"use_sim_time": True}],
    #)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        #rviz_node,
    ])
