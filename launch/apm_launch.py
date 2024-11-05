from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mavros_dir = get_package_share_directory('mavros')
    param_file = os.path.join(mavros_dir, 'config', 'apm_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='tcp://localhost',
            description='URL for FCU connection'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='udp://@',
            description='URL for GCS connection'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='MAVLink target system ID'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='MAVLink target component ID'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': LaunchConfiguration('gcs_url')},
                {'target_system_id': LaunchConfiguration('tgt_system')},
                {'target_component_id': LaunchConfiguration('tgt_component')},
                param_file
            ],
        )
    ])

