from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. Start device nodes first (GPIO, Motor, Laser)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('device'),
                    'launch',
                    'device.launch.py'
                ])
            ])
        ),
        
        # 2. Start camera after devices are ready
        TimerAction(
            period=8.0,  # Wait 8 seconds for devices to stabilize
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('camera'),
                            'launch',
                            'camera.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'enable_display': 'false',  # Backend mode
                        'enable_web_stream': 'true',  # Enable web streaming
                        'web_port': '8080',
                        'input_mode': 'webcam'
                    }.items()
                )
            ]
        ),
        
        # 3. Start robot controller after camera is ready
        TimerAction(
            period=12.0,  # Wait 12 seconds total
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('robot'),
                            'launch',
                            'robot.launch.py'
                        ])
                    ])
                )
            ]
        )
    ])