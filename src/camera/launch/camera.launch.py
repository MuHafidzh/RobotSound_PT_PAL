from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_mode',
            default_value='webcam',
            description='Input mode: webcam, topic, or image'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/pal/pal_ws/src/camera/models',
            description='Path to YOLO model files'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='/home/pal/pal_ws/src/camera/config/color_config.yaml',
            description='Path to color detection config'
        ),
        DeclareLaunchArgument(
            'enable_display',
            default_value='false',
            description='Enable OpenCV display windows'
        ),
        DeclareLaunchArgument(
            'enable_web_stream',
            default_value='true',
            description='Enable web video streaming'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web server port'
        ),
        DeclareLaunchArgument(
            'image_path',
            default_value='',
            description='Path to image file (for image mode)'
        ),
        
        # YOLO Node
        Node(
            package='camera',
            executable='yolo_node',
            name='yolo_node',
            output='log',
            parameters=[
                {'input_mode': LaunchConfiguration('input_mode')},
                {'model_path': LaunchConfiguration('model_path')},
                {'config_path': LaunchConfiguration('config_path')},
                {'enable_display': LaunchConfiguration('enable_display')},
                {'image_path': LaunchConfiguration('image_path')},
                {'camera_device': 0}
            ],
            respawn=True,
            respawn_delay=3.0
        ),
        
        # Web Video Server Node (conditional)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='log',
            parameters=[
                {'port': LaunchConfiguration('web_port')},
                {'default_stream_type': 'mjpeg'},
                {'quality': 80}
            ],
            condition=IfCondition(LaunchConfiguration('enable_web_stream')),  # PERBAIKAN: Gunakan IfCondition
            respawn=True,
            respawn_delay=2.0
        )
    ])