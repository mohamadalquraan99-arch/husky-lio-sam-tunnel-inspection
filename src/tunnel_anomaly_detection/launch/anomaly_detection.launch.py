from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('tunnel_anomaly_detection')
    config_file = os.path.join(package_share, 'config', 'anomaly_detection.yaml')

    baseline_argument = DeclareLaunchArgument(
        'baseline_pcd',
        default_value=(
            '/home/momo1/husky_ws/inspection_data/'
            'baseline_01/lio_sam_3d/GlobalMap.pcd'
        ),
        description='Absolute path to the baseline GlobalMap.pcd file',
    )

    current_argument = DeclareLaunchArgument(
        'current_pcd',
        default_value='',
        description=(
            'Optional current GlobalMap.pcd. When provided, compare the two '
            'saved dense maps instead of subscribing to the live topic.'
        ),
    )

    detector = Node(
        package='tunnel_anomaly_detection',
        executable='tunnel_anomaly_detector',
        name='tunnel_anomaly_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'baseline_pcd': LaunchConfiguration('baseline_pcd'),
                'current_pcd': LaunchConfiguration('current_pcd'),
            },
        ],
    )

    return LaunchDescription([baseline_argument, current_argument, detector])
