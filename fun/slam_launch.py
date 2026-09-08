#!/usr/bin/env python3
"""手持式 2D 激光 SLAM 启动文件。

整合以下节点，实现无里程计（手持）场景下的同步定位与建图：
  1. rplidar_node         —— 雷达驱动，发布 /scan（frame_id=laser）
  2. static_transform      —— 发布 base_link -> laser 的静态 TF
  3. laser_scan_matcher    —— 由激光估算里程计，发布 odom -> base_link TF
  4. slam_toolbox          —— 在线异步 SLAM，发布 map -> odom，输出 /map
  5. rviz2                 —— 可视化地图与激光

TF 树：map -> odom -> base_link -> laser
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(pkg_dir, "config")

    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=os.path.join(config_dir, "mapper_params_online_async.yaml"),
    )
    rviz_config_file = LaunchConfiguration(
        "rviz_config_file",
        default=os.path.join(config_dir, "slam.rviz"),
    )
    autostart = LaunchConfiguration("autostart", default="true")

    # 1. 雷达驱动节点
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": 460800,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        output="screen",
    )

    # 2. base_link -> laser 静态 TF（手持式，雷达即本体，偏移为 0）
    base_to_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["0", "0", "0", "0", "0", "0",
                   "base_link", "laser"],
        output="screen",
    )

    # 3. 激光里程计（laser_scan_matcher）
    laser_scan_matcher_node = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher",
        parameters=[{
            "base_frame": "base_link",
            "odom_frame": "odom",
            "laser_frame": "laser",
            "publish_tf": True,
            "publish_odom": "",
            "kf_dist_linear": 0.05,
            "kf_dist_angular": 0.2,
            "max_angular_correction_deg": 45.0,
            "max_linear_correction": 0.5,
            "max_iterations": 10,
            "max_correspondence_dist": 0.3,
        }],
        output="screen",
    )

    # 4. slam_toolbox 在线异步 SLAM（lifecycle 节点）
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_params_file,
            {"use_sim_time": False},
        ],
        output="screen",
        namespace="",
    )

    # 生命周期自动启动：configure -> activate
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Slamtoolbox node is activating."
                ),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    # 5. rviz2 可视化
    # 树莓派 v3d 驱动仅支持 OpenGL 3.1，rviz2 地图 shader 会导致崩溃，
    # 故强制使用软件渲染（llvmpipe，OpenGL 4.5）
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value=serial_port,
            description="雷达串口设备路径"),

        DeclareLaunchArgument(
            "slam_params_file", default_value=slam_params_file,
            description="slam_toolbox 参数文件路径"),

        DeclareLaunchArgument(
            "rviz_config_file", default_value=rviz_config_file,
            description="rviz2 配置文件路径"),

        rplidar_node,
        base_to_laser_node,
        laser_scan_matcher_node,
        slam_toolbox_node,
        configure_event,
        activate_event,
        rviz2_node,
    ])
