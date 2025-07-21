from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Auto mode parameters
        DeclareLaunchArgument(
            'target_distance',
            default_value='1.0',
            description='Target following distance in meters'
        ),
        DeclareLaunchArgument(
            'min_distance',
            default_value='0.5',
            description='Minimum safety distance in meters'
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.2',
            description='Maximum linear speed in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='2.5',
            description='Maximum angular speed in rad/s'
        ),
        DeclareLaunchArgument(
            'detection_timeout',
            default_value='10.0',
            description='Person detection timeout in seconds'
        ),
        DeclareLaunchArgument(
            'search_duration',
            default_value='16.0',
            description='Search mode duration in seconds'
        ),
        DeclareLaunchArgument(
            'search_angular_speed',
            default_value='2.2',
            description='Angular speed during search in rad/s'
        ),

        # Gain parameters untuk tuning
        DeclareLaunchArgument(
            'angular_gain',
            default_value='0.01',
            description='Angular velocity gain for person tracking'
        ),
        DeclareLaunchArgument(
            'linear_gain',
            default_value='0.01',
            description='Linear velocity gain for distance control'
        ),
        DeclareLaunchArgument(
            'control_threshold_cm',
            default_value='5.0',
            description='Distance control dead zone in cm'
        ),
        DeclareLaunchArgument(
            'max_control_distance_cm',
            default_value='400.0',
            description='Maximum control distance in cm'
        ),
        
        # Camera parameters
        DeclareLaunchArgument(
            'camera_width',
            default_value='480',
            description='Camera image width'
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='360',
            description='Camera image height'
        ),
        
        # Robot Main Controller Node
        Node(
            package='robot',
            executable='main_node',
            name='robot_main_node',
            output='screen',
            parameters=[
                {'auto.target_distance': LaunchConfiguration('target_distance')},
                {'auto.min_distance': LaunchConfiguration('min_distance')},
                {'auto.max_linear_speed': LaunchConfiguration('max_linear_speed')},
                {'auto.max_angular_speed': LaunchConfiguration('max_angular_speed')},
                {'auto.detection_timeout': LaunchConfiguration('detection_timeout')},
                {'auto.search_duration': LaunchConfiguration('search_duration')},
                {'auto.search_angular_speed': LaunchConfiguration('search_angular_speed')},
                {'auto.angular_gain': LaunchConfiguration('angular_gain')},
                {'auto.linear_gain': LaunchConfiguration('linear_gain')},
                {'auto.control_threshold_cm': LaunchConfiguration('control_threshold_cm')},
                {'auto.max_control_distance_cm': LaunchConfiguration('max_control_distance_cm')},
                {'camera.width': LaunchConfiguration('camera_width')},
                {'camera.height': LaunchConfiguration('camera_height')}
            ],
            respawn=True,
            respawn_delay=2.0
        )
    ])