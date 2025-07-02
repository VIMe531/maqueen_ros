from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('maqueen_auto'), 'config')

    return LaunchDescription([
        Node(
			package='maqueen_auto',
			executable='scan_timesync_node',
			name='scan_timesync',
			output='screen',
		),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='maqueen_cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename',  'cartographer.lua',
            ],
            remappings=[('scan', 'scan_timesynced')],
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='maqueen_occupancy_grid_node',
         	arguments=[
        		'-resolution',       '0.05',
        		'-publish_period_sec','1.0',
    		],
            output='screen',
        ),
#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            name='static_map_to_odom',
#            arguments=[
#                '0','0','0',
#                '0','0','0',
#                'map','odom'
#            ],
#            output='screen',
#        ),
    ])

