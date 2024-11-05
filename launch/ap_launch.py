import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/cuav_v5_nano:115200',
        description='URL for the FCU connection'
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='URL for the GCS connection'
    )

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': 1,
            'target_component_id': 1,
        }],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        mavros_node
    ])
