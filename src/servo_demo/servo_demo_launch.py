import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Explicitly specify the package name found in your install folder
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda", 
            package_name="moveit_resources_panda_moveit_config"
        )
        .to_moveit_configs()
    )

    # Start your custom C++ node
    servo_demo_node = Node(
        package='servo_demo',
        executable='servo_cpp_interface_demo',
        name='servo_demo_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # This ensures the 'move_group_name' (panda_arm) is loaded
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([servo_demo_node])