from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='127.0.0.1',
            description='TCP server IP'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='30004',
            description='TCP server port'
        ),
        Node(
            package='aubo_ros2_driver',
            executable='aubo_client_node.py',
            name='aubo_client',
            output='screen',
            parameters=[{
                'tcp_client.ip': LaunchConfiguration('ip'),
                'tcp_client.port': LaunchConfiguration('port'),
            }]
        )
    ])
