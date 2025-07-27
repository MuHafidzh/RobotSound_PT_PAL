from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'laser_port',
            default_value='/dev/ttyAMA0',
            description='Serial port for laser sensor'
        ),
        DeclareLaunchArgument(
            'laser_baudrate',
            default_value='115200',
            description='Baudrate for laser sensor'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface for motor driver'
        ),
        DeclareLaunchArgument(
            'enable_gpio',
            default_value='false',
            description='Enable GPIO control for motor driver'
        ),
        DeclareLaunchArgument(
            'enable_joyandro',
            default_value='true',
            description='Enable JoyAndro bluetooth controller'
        ),
        
        # GPIO Node - Start first (fastest to initialize)
        Node(
            package='device',
            executable='gpio_node',
            name='gpio_node',
            output='log',
            parameters=[],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_gpio'))
        ),
        
        # JoyAndro Node - Start after GPIO
        TimerAction(
            period=1.0,  # Wait 1 second after GPIO
            actions=[
                Node(
                    package='device',
                    executable='joyandro_node',
                    name='joyandro_node',
                    output='log',
                    parameters=[],
                    respawn=True,
                    respawn_delay=3.0,
                    condition=IfCondition(LaunchConfiguration('enable_joyandro'))
                )
            ]
        ),
        
        # Motor Driver Node - Start after GPIO
        TimerAction(
            period=2.0,  # Wait 2 seconds after GPIO
            actions=[
                Node(
                    package='device',
                    executable='zlac8015_node',
                    name='zlac8015_node',
                    output='log',
                    parameters=[
                        {'can_interface': LaunchConfiguration('can_interface')},
                        {'wheel_base': 0.3},
                        {'wheel_radius': 0.107},
                        {'encoder_cpr': 4096}
                    ],
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        ),
        
        # Laser Node - Start after motor
        TimerAction(
            period=5.0,  # Wait 5 seconds after GPIO
            actions=[
                Node(
                    package='device',
                    executable='laser_node',
                    name='laser_node',
                    output='log',
                    parameters=[
                        {'port': LaunchConfiguration('laser_port')},
                        {'baudrate': LaunchConfiguration('laser_baudrate')},
                        {'slave_id': 1},
                        {'frequency': 50.0}
                    ],
                    respawn=True,
                    respawn_delay=3.0
                )
            ]
        )
    ])