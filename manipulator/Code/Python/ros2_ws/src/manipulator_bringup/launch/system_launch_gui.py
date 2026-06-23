from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    homing_node = Node(
        package='manipulator_homing',
        executable='homing_node'
    )

    servo_node = Node(
        package='manipulator_servos',
        executable='servo_node'
    )

    ik_node = Node(
        package='manipulator_ik',
        executable='ik_node'
    )
    

    return LaunchDescription([
        homing_node,
        servo_node,
        ik_node,
    ])