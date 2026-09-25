"""ROS2 launch file for VINS-Fusion loop_fusion (loop closure + pose graph).

`loop_fusion_node` 消费 `vins_node` 输出的话题，做 DBoW2 回环检测 + 位姿图优化，
并把修正后的轨迹发布到 `/pose_graph/*`。

用法::

    # 1) 先起 VIO
    ros2 launch vins euroc.launch.py

    # 2) 再起回环检测（与 vins 共用同一份 config）
    ros2 launch loop_fusion loop_fusion.launch.py

    # 换数据集 / 换配置
    ros2 launch loop_fusion loop_fusion.launch.py \\
        config_path:=<ws>/install/vins/share/vins/config/kitti_odom/kitti_config00-02.yaml

    # 连 RViz 一起起
    ros2 launch loop_fusion loop_fusion.launch.py rviz:=true

注意：

* `loop_fusion_node` 要求 **恰好一个** 命令行参数——配置文件路径
  (`pose_graph_node.cpp:426-432`，`argc != 2` 会直接打印用法并退出)。
* 配置文件里 `cam0_calib` 是**相对于配置文件所在目录**解析的
  (`pose_graph_node.cpp:461-467`)，所以不能把 yaml 单独拷出来用。
* 词袋 `brief_k10L6.bin` 与 `brief_pattern.yml` 由 `pose_graph_node.cpp:453-459`
  按 `get_package_share_directory("loop_fusion") + "/../support_files/"` **硬编码**定位，
  对应的安装规则见 `loop_fusion/CMakeLists.txt`。这两个文件缺失会导致节点崩溃。
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_path():
    """默认复用 vins 安装目录里的 EuRoC 双目+IMU 配置。

    loop_fusion 和 vins 必须用**同一份**传感器配置，否则相机内参、图像尺寸、
    话题名会对不上。找不到 vins 包时返回空串，交给下面的校验报错。
    """
    try:
        vins_share = get_package_share_directory('vins')
    except PackageNotFoundError:
        return ''
    return os.path.join(
        vins_share, 'config', 'euroc', 'euroc_stereo_imu_config.yaml')


def _resolve_rviz_config():
    """vins_rviz_config.rviz 里已经预置了 /pose_graph/* 的显示项，直接复用。"""
    try:
        return os.path.join(
            get_package_share_directory('vins'), 'config', 'vins_rviz_config.rviz')
    except PackageNotFoundError:
        return ''


def _launch_setup(context, *args, **kwargs):
    """校验配置路径，然后组装节点。"""
    config_path = LaunchConfiguration('config_path').perform(context)

    if not config_path or not os.path.isfile(config_path):
        install_hint = os.path.join(
            '<ws>', 'install', 'vins', 'share', 'vins',
            'config', 'euroc', 'euroc_stereo_imu_config.yaml')
        raise RuntimeError(
            '\n[loop_fusion] 配置文件不可用: {path!r}\n'
            '  loop_fusion_node 必须收到一个**存在**的 config yaml（它同时用来定位\n'
            '  同目录下的相机标定文件）。请显式指定，例如:\n'
            '    ros2 launch loop_fusion loop_fusion.launch.py '
            'config_path:={hint}\n'
            '  可用配置见 <ws>/install/vins/share/vins/config/'.format(
                path=config_path, hint=install_hint))

    # 词袋与 BRIEF pattern 的硬编码位置，缺失时提前给出可读的报错。
    try:
        loop_share = get_package_share_directory('loop_fusion')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            '\n[loop_fusion] 找不到 loop_fusion 包。请先 source 工作空间:\n'
            '    source <ws>/install/local_setup.bash') from exc

    support_dir = os.path.normpath(os.path.join(loop_share, os.pardir, 'support_files'))
    missing = [
        name for name in ('brief_k10L6.bin', 'brief_pattern.yml')
        if not os.path.isfile(os.path.join(support_dir, name))
    ]
    if missing:
        raise RuntimeError(
            '\n[loop_fusion] 词袋文件缺失: {files}\n'
            '  期望位置: {dir}\n'
            '  pose_graph_node.cpp 按 "<loop_fusion share>/../support_files/" 硬编码查找。\n'
            '  源码树里这两个文件位于 <repo>/support_files/，请重新编译以触发安装:\n'
            '    colcon build --packages-select loop_fusion'.format(
                files=', '.join(missing), dir=support_dir))

    loop_fusion_node = Node(
        package='loop_fusion',
        executable='loop_fusion_node',
        name='loop_fusion',
        output='screen',
        arguments=[config_path],
        remappings=[
            # —— 输出：与 vins_rviz_config.rviz 里的 display 话题保持一致 ——
            ('pose_graph_path',        '/pose_graph/pose_graph_path'),
            ('base_path',              '/pose_graph/base_path'),
            ('pose_graph',             '/pose_graph/pose_graph'),
            ('camera_pose_visual',     '/pose_graph/camera_pose_visual'),
            ('match_image',            '/pose_graph/match_image'),
            ('odometry_rect',          '/pose_graph/odometry_rect'),
            ('path_1',                 '/pose_graph/path_1'),
            ('path_2',                 '/pose_graph/path_2'),
            ('path_3',                 '/pose_graph/path_3'),
            ('path_4',                 '/pose_graph/path_4'),
            ('path_5',                 '/pose_graph/path_5'),
            ('path_6',                 '/pose_graph/path_6'),
            ('path_7',                 '/pose_graph/path_7'),
            ('path_8',                 '/pose_graph/path_8'),
            ('path_9',                 '/pose_graph/path_9'),
            # 注意: point_cloud_loop_rect / margin_cloud_loop_rect 保持根命名空间，
            # 因为 rviz 配置里引用的就是 /point_cloud_loop_rect。
            #
            # 输入话题（/vins_estimator/odometry、keyframe_pose、keyframe_point、
            # extrinsic、margin_cloud，以及来自 config 的 image0_topic）已经是
            # 绝对路径，默认无需重映射；若要接非 vins 的里程计源，在此追加即可。
        ],
    )

    nodes = [loop_fusion_node]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_loop_fusion',
        output='log',
        arguments=['-d', _resolve_rviz_config()],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=_default_config_path(),
            description='传感器/相机配置 yaml（须与 vins 使用同一份；cam0_calib 相对其所在目录解析）'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='是否同时启动 RViz2（加载 vins 的 rviz 配置，含 /pose_graph/* 显示项）'),
        OpaqueFunction(function=_launch_setup),
    ])
