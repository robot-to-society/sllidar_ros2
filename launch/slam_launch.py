import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource

import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('sllidar_ros2')
    mavros_dir = get_package_share_directory('mavros')
    rviz_config_file = os.path.join(share_dir, 'rviz','sllidar_ros2.rviz')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='sllidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY', 'ERROR'),

	# RPLidar A3
	IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(share_dir, 'launch/sllidar_a3_launch.py')
		),
                launch_arguments={'frame_id': 'laser'}.items()
	),

        # MAVROS
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/cuav_v5_nano:115200',
            description='URL for FCU connection'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='udp://@192.168.2.20:14550',
            description='URL for GCS connection'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVLink 2.0'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'fcu_protoco': LaunchConfiguration('fcu_protocol'),
                'gcs_url': LaunchConfiguration('gcs_url'),
            }.items()
        ),

        Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
            ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
            ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # map TF to odom TF
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),

        #Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # odom TF to base_footprint
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    namespace='',
        #    output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
        #),

        TimerAction(
		period=30.0,
                actions=[
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='log',
            parameters=[
                {'use_sim_time': use_sim_time},
                #{'tf_buffer_duration': 60.0}
            ],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[
                ('odom','/mavros/local_position/odom'),
                ('imu','/mavros/imu/data'),
            ]
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
        ]),

	# Robot pose publisher
	Node(
	    package='robot_pose_publisher_ros2',
	    executable='robot_pose_publisher',
	    name='robot_pose_publisher',
	    output='screen',
	    parameters=[
		{'use_sim_time': use_sim_time},
		{'is_stamped': True},
		{'map_frame': 'map'},
		{'base_frame': 'base_link'}
	    ],
            remappings=[
                ('robot_pose','/mavros/vision_pose/pose')
            ]
	),
            
    ])

