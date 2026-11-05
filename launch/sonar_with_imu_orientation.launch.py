import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'sonar_frame_id', default_value='sonar_link',
            description='Point cloud frame; overrides frame_id in the sonar YAML'),
        DeclareLaunchArgument(
            'imu_frame_id', default_value='sonar_imu_link',
            description='Raw IMU frame; overrides imu_frame_id in the sonar YAML'),
        DeclareLaunchArgument(
            'publish_mount_tf', default_value='true',
            description='Publish sonar-to-IMU TF; disable if supplied by your robot URDF'),
        *[
            DeclareLaunchArgument(name, default_value=value, description=description)
            for name, value, description in [
                ('mount_x', '-0.022', 'IMU origin X in the sonar frame, meters'),
                ('mount_y', '-0.046', 'IMU origin Y in the sonar frame, meters'),
                ('mount_z', '-0.003', 'IMU origin Z in the sonar frame, meters'),
                ('mount_roll', '0.0', 'IMU roll relative to sonar, radians'),
                ('mount_pitch', '0.0', 'IMU pitch relative to sonar, radians'),
                ('mount_yaw', '0.0', 'IMU yaw relative to sonar, radians'),
            ]
        ],
        Node(
            package='waterlinked_sonar_3d15',
            executable='sonar_node',
            name='sonar_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'frame_id': LaunchConfiguration('sonar_frame_id'),
                    'imu_frame_id': LaunchConfiguration('imu_frame_id'),
                },
            ],
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sonar_to_imu_static_tf',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
            arguments=[
                '--x', LaunchConfiguration('mount_x'),
                '--y', LaunchConfiguration('mount_y'),
                '--z', LaunchConfiguration('mount_z'),
                '--roll', LaunchConfiguration('mount_roll'),
                '--pitch', LaunchConfiguration('mount_pitch'),
                '--yaw', LaunchConfiguration('mount_yaw'),
                '--frame-id', LaunchConfiguration('sonar_frame_id'),
                '--child-frame-id', LaunchConfiguration('imu_frame_id'),
            ],
            output='screen',
        ),
    ])
