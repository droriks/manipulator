from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    height_center_arg = DeclareLaunchArgument('height_center', default_value='107.5')
    height_amplitude_arg = DeclareLaunchArgument('height_amplitude', default_value='0.0')
    height_offset_arg = DeclareLaunchArgument('height_offset', default_value='0.0')
    psi_center_arg = DeclareLaunchArgument('psi_center', default_value='0.0')
    psi_amplitude_arg = DeclareLaunchArgument('psi_amplitude', default_value='0.0')
    psi_offset_arg = DeclareLaunchArgument('psi_offset', default_value='0.0')
    theta_center_arg = DeclareLaunchArgument('theta_center', default_value='0.0')
    theta_amplitude_arg = DeclareLaunchArgument('theta_amplitude', default_value='0.0')
    theta_offset_arg = DeclareLaunchArgument('theta_offset', default_value='0.0')
    period_arg = DeclareLaunchArgument('period', default_value='2.0')
    publish_rate_arg = DeclareLaunchArgument('publish_rate', default_value='.009')
    
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
    
    sine_wave_node = Node(
        package='manipulator_trajectory',
        executable='sine_wave_node',
        parameters=[{
            'height_center' : LaunchConfiguration('height_center'),
            'height_amplitude' : LaunchConfiguration('height_amplitude'),
            'height_offset' : LaunchConfiguration('height_offset'),
            'psi_center' : LaunchConfiguration('psi_center'),
            'psi_amplitude' : LaunchConfiguration('psi_amplitude'),
            'psi_offset' : LaunchConfiguration('psi_offset'),
            'theta_center' : LaunchConfiguration('theta_center'),
            'theta_amplitude' : LaunchConfiguration('theta_amplitude'),
            'theta_offset' : LaunchConfiguration('theta_offset'),
            'period' : LaunchConfiguration('period'),
            'publish_rate' : LaunchConfiguration('publish_rate')
            
        }]
    )

    return LaunchDescription([
        height_center_arg,
        height_amplitude_arg,
        height_offset_arg,
        psi_center_arg,
        psi_amplitude_arg,
        psi_offset_arg,
        theta_center_arg,
        theta_amplitude_arg,
        theta_offset_arg,
        period_arg,
        publish_rate_arg,
        homing_node,
        servo_node,
        ik_node,
        sine_wave_node
    ])