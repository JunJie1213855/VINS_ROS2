"""ROS2 launch file for VINS-Fusion on EuRoC datasets.

默认同时启动 `vins_node` 与 RViz2（用 `config/vins_rviz_config.rviz`）。

用法::

    # VIO + RViz（默认）
    ros2 launch vins euroc.launch.py

    # 只要 VIO，不要可视化（纯 ssh / 机器人本体等无显示环境）
    ros2 launch vins euroc.launch.py rviz:=false

    # 换配置
    ros2 launch vins euroc.launch.py config_path:=<ws>/install/vins/share/vins/config/realsense_d435i/realsense_stereo_imu_config.yaml

提示：rviz 配置里 `/pose_graph/*` 那一组回环显示默认是**关闭**的
（`vins_rviz_config.rviz` 里 `Name: pose_graph` 的 `Enabled: false`）。
把 `loop_fusion/launch/loop_fusion.launch.py` 也起起来后，
再在 RViz 左侧 Displays 面板勾上该组即可看到闭环优化后的轨迹/点云。

只想单独起可视化（VIO 已在别处运行）时仍可用 `vins_rviz.launch.py`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vins')

    default_config = os.path.join(
        pkg_share, 'config', 'euroc', 'euroc_stereo_imu_config.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'vins_rviz_config.rviz')

    config_path_arg = DeclareLaunchArgument(
        'config_path', default_value=default_config,
        description='Path to config YAML file')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否同时启动 RViz2（无显示环境请传 rviz:=false）')

    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        arguments=[LaunchConfiguration('config_path')],
        remappings=[
            ('imu_propagate',      '/vins_estimator/imu_propagate'),
            ('path',               '/vins_estimator/path'),
            ('odometry',           '/vins_estimator/odometry'),
            ('point_cloud',        '/vins_estimator/point_cloud'),
            ('margin_cloud',       '/vins_estimator/margin_cloud'),
            ('key_poses',          '/vins_estimator/key_poses'),
            ('camera_pose',        '/vins_estimator/camera_pose'),
            ('camera_pose_visual', '/vins_estimator/camera_pose_visual'),
            ('keyframe_pose',      '/vins_estimator/keyframe_pose'),
            ('keyframe_point',     '/vins_estimator/keyframe_point'),
            ('extrinsic',          '/vins_estimator/extrinsic'),
            ('image_track',        '/vins_estimator/image_track'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        config_path_arg,
        rviz_arg,
        vins_node,
        rviz_node,
    ])
