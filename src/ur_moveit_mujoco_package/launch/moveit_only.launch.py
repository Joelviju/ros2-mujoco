from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="ur_moveit_mujoco_package")
        .robot_description(file_path="config/ur10e.urdf.xacro")
        .robot_description_semantic(file_path="config/ur10e.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # ❌ NO planning_pipelines() → defaults to CHOMP
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # move_group (planner + execution brain)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # Robot State Publisher (TF + robot_description)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
    ])
