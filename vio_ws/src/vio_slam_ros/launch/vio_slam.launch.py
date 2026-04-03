#!/usr/bin/env python3
"""VIO-SLAM launch file for monocular configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='euroc_mono.yaml',
        description='Configuration file name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Topics
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='Input IMU topic'
    )
    
    # VIO-SLAM node
    vio_slam_node = Node(
        package='vio_slam_ros',
        executable='vio_slam_node',
        name='vio_slam',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('vio_slam_ros'),
                'config',
                LaunchConfiguration('config')
            ]),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_topic': LaunchConfiguration('image_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }
        ],
        remappings=[
            ('~/image_raw', LaunchConfiguration('image_topic')),
            ('~/imu_raw', LaunchConfiguration('imu_topic')),
        ]
    )
    
    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        image_topic_arg,
        imu_topic_arg,
        vio_slam_node,
    ])
