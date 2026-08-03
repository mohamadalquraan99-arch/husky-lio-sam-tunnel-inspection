from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_mode = LaunchConfiguration('mission_mode')
    data_root = LaunchConfiguration('data_root')
    baseline_id = LaunchConfiguration('baseline_id')
    mission_id = LaunchConfiguration('mission_id')

    arguments = [
        DeclareLaunchArgument(
            'mission_mode', default_value='inspection',
            description="Choose 'baseline' or 'inspection'"),
        DeclareLaunchArgument(
            'data_root', default_value='/home/momo1/husky_ws/inspection_data'),
        DeclareLaunchArgument('baseline_id', default_value='baseline_01'),
        DeclareLaunchArgument('mission_id', default_value='mission_03'),
    ]

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('husky_tunnel_bringup'),
                'launch',
                'tunnel_backtracking_exploration.launch.py',
            ])))

    manager = Node(
        package='tunnel_mission_manager',
        executable='mission_manager',
        name='tunnel_mission_manager',
        output='screen',
        parameters=[{
            'mission_mode': mission_mode,
            'data_root': data_root,
            'baseline_id': baseline_id,
            'mission_id': mission_id,
            'map_resolution': 0.1,
            'overwrite_output': False,
            'use_sim_time': True,
        }])

    anomaly_config = PathJoinSubstitution([
        FindPackageShare('tunnel_anomaly_detection'),
        'config',
        'anomaly_detection.yaml',
    ])
    baseline_pcd = PathJoinSubstitution([
        data_root, baseline_id, 'lio_sam_3d', 'GlobalMap.pcd',
    ])

    detector = Node(
        package='tunnel_anomaly_detection',
        executable='tunnel_anomaly_detector',
        name='tunnel_anomaly_detector',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mission_mode, "' == 'inspection'",
        ])),
        parameters=[
            anomaly_config,
            {
                'baseline_pcd': baseline_pcd,
                'current_pcd': '',
                'wait_for_trigger': True,
                'use_icp_alignment': True,
                'change_distance_threshold_m': 0.35,
                'process_once': True,
                'use_sim_time': True,
            },
        ])

    return LaunchDescription(arguments + [exploration, manager, detector])
