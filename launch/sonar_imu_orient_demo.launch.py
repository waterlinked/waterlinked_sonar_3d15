import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_FILE = 'default_params.yaml'
IMU_FILTER_CONFIG_FILE = 'imu_filter_params.yaml'


def generate_launch_description():
    pkg_share = get_package_share_directory('waterlinked_sonar_3d15')
    default_params_file = os.path.join(pkg_share, 'config', CONFIG_FILE)
    default_filter_params_file = os.path.join(pkg_share, 'config', IMU_FILTER_CONFIG_FILE)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the sonar parameter YAML file',
        ),
        DeclareLaunchArgument(
            'imu_filter_params_file',
            default_value=default_filter_params_file,
            description='Full path to the IMU filter parameter YAML file',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Node namespace',
        ),
        Node(
            package='waterlinked_sonar_3d15',
            executable='sonar_node',
            name='sonar_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('imu_filter_params_file')],
            remappings=[
                ('imu/data_raw', 'sonar_node/imu'),
                ('imu/data', 'imu/data'),
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])