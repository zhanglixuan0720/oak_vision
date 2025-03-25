from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640', description='Image width'),
        DeclareLaunchArgument('height', default_value='400', description='Image height'),
        DeclareLaunchArgument('fps', default_value='30', description='Frames per second'),
        DeclareLaunchArgument('mask_vio', default_value='False', description='Mask visual odometry'),
        
        Node(
            package='oak_vision',
            executable='oak_vision_node',
            name='oak_camera',
            output='screen',
            parameters=[{
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'mask_vio': LaunchConfiguration('mask_vio')
            }]
        )
    ])
