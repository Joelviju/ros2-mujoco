cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_bridge)

# ------------------------------------------------
# Compiler
# ------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ------------------------------------------------
# ROS 2 core
# ------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# ------------------------------------------------
# Messages / Actions
# ------------------------------------------------
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


# ------------------------------------------------
# MoveIt
# ------------------------------------------------
find_package(moveit_ros_planning_interface REQUIRED)

# ------------------------------------------------
# MuJoCo
# ------------------------------------------------
if(NOT DEFINED ENV{MUJOCO_HOME})
  message(FATAL_ERROR "MUJOCO_HOME environment variable is not set")
endif()

set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_HOME}/include)
set(MUJOCO_LIBRARY $ENV{MUJOCO_HOME}/lib/libmujoco.so)

if(NOT EXISTS ${MUJOCO_LIBRARY})
  message(FATAL_ERROR "MuJoCo library not found at ${MUJOCO_LIBRARY}")
endif()

# ------------------------------------------------
# GLFW / OpenGL
# ------------------------------------------------
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# ------------------------------------------------
# Executable: MuJoCo FollowJointTrajectory server
# ------------------------------------------------
add_executable(mujoco_follow_joint_trajectory
  src/mujoco_follow_joint_trajectory.cpp
)

target_include_directories(mujoco_follow_joint_trajectory PUBLIC
  ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(mujoco_follow_joint_trajectory
  ${MUJOCO_LIBRARY}
  glfw
  OpenGL::GL
)

ament_target_dependencies(mujoco_follow_joint_trajectory
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
 
)

# ------------------------------------------------
# Executable: UR10e MoveIt client
# ------------------------------------------------
add_executable(ur10e_moveit_robot
  src/ur10e_moveit_robot.cpp
)

ament_target_dependencies(ur10e_moveit_robot
  rclcpp
  moveit_ros_planning_interface
)

# ------------------------------------------------
# Install
# ------------------------------------------------
install(TARGETS
  mujoco_follow_joint_trajectory
  ur10e_moveit_robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
