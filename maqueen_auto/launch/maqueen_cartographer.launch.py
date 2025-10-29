# maqueen_cartographer.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('maqueen_auto')

    # Launch arguments
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename  = LaunchConfiguration('configuration_basename')
    use_sim_time            = LaunchConfiguration('use_sim_time')
    resolution              = LaunchConfiguration('resolution')
    publish_period_sec      = LaunchConfiguration('publish_period_sec')

    rviz_config_file = os.path.join(share_dir, 'config', 'maqueen_nav.rviz')

    declare_args = [
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(share_dir, 'config'),
            description='Directory that contains cartographer.lua'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Basename of the lua file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated time if true'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy grid resolution (m)'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Occupancy grid publish period (sec)'
        ),
    ]

    # scan timestamp resync (micro-ROS sensor timestamps can drift)
    scan_timesync_node = Node(
        package='maqueen_auto',
        executable='scan_timesync_node',
        name='scan_timesync',
        output='screen'
    )

    # static TF: base_link -> laser_frame（Cartographerより先に上げる）
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='maqueen_cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename',  configuration_basename
        ],
        remappings=[('scan', 'scan_timesynced')]
    )

    # Occupancy grid publisher
    grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription(declare_args + [
        scan_timesync_node,
        tf_laser_node,
        cartographer_node,
        grid_node,
        rviz2_node,
    ])
