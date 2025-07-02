from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maqueen_tf',
            executable='maqueen_lite_tf_node',
            name='maqueen_lite_tf',
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_pub',
            arguments=['0.1', '0.0', '0.05', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen',
        ),
    ])

