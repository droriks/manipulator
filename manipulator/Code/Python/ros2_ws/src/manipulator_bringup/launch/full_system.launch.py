from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value = '/home/hydro/dev/manipulator/manipulator/Code/Python/ros2_ws/src/manipulator_ik/data/sine_table.csv',
        description = 'Path to precomputed trajectory csv file'
    )

    servo_node = Node(
        package = 'manipulator_servos',
        executable = 'servo_node'
    )

    homing_node = Node(
        package = 'manipulator_homing',
        executable = 'homing_node'
    )

    playback_node = Node(
        package = 'manipulator_trajectory',
        executable = 'playback_node',
        parameters = [{
            'csv_path': LaunchConfiguration('csv_path')
        }]
    )
    
    return LaunchDescription([
        csv_path_arg,
        servo_node,
        homing_node,
        playback_node
    ])