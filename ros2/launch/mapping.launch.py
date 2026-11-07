#!/usr/bin/env python3
"""地図作りモードの起動ファイル（ROS 2 Jazzy / Docker の部屋の中で使う）

起動するもの
  1. LD19 のドライバー（後ろ側の車体が映る範囲を消して /scan を出す）
  2. センサーの取り付け位置（base_link → base_laser / imu_link）
  3. Cartographer（LiDAR だけで地図を作る。車輪の回転情報は使わない）
  4. 地図の画像を作るノード（/map を出す）
  5. LiDAR の生データの記録（あとで地図を作り直せるように）

使い方（部屋の中で）
  ros2 launch /workspace/ros2/launch/mapping.launch.py map_name:=room

  map_name   : 生データの保存名（/maps/<map_name>_<日時>_bag に保存）
  record     : 生データを記録するか（true / false、初期値 true）
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.normpath(os.path.join(HERE, '..', 'config'))

# ===== 調整できる値（docs/PLAN.md と合わせる）=====

# センサーの取り付け位置 [m]、向き [rad]
# 基準点 base_link ＝ 後輪の車軸の中心の真下の床。前 +x、左 +y、上 +z
LIDAR_X, LIDAR_Y, LIDAR_Z, LIDAR_YAW = 0.225, 0.0, 0.115, 0.0   # 実測では約 +2°（0.035）ずれている
IMU_X, IMU_Y, IMU_Z, IMU_YAW = 0.170, 0.0, 0.037, 1.5708        # センサーの −Y が車の前なので +90°

# 無視する角度の範囲 [°]（後ろのマウントの足・ケーブルが映る範囲）
# 大きく取るほど車体を拾わなくなるが、見える範囲が狭くなる
IGNORE_MIN_DEG = 150.0
IGNORE_MAX_DEG = 210.0

# LD19 の接続先
LIDAR_PORT = '/dev/ttyUSB0'

# 地図の細かさ [m/マス]。小さくするほど細かいが重くなる
MAP_RESOLUTION = 0.05


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    record = LaunchConfiguration('record')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'base_laser',
            'port_name': LIDAR_PORT,
            'port_baudrate': 230400,
            'laser_scan_dir': True,              # 反時計回り（ROS 2 の標準）
            'enable_angle_crop_func': True,      # 下の範囲の点を 0 にして消す
            'angle_crop_min': IGNORE_MIN_DEG,
            'angle_crop_max': IGNORE_MAX_DEG,
        }],
    )

    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_base_laser',
        arguments=[
            '--x', str(LIDAR_X), '--y', str(LIDAR_Y), '--z', str(LIDAR_Z),
            '--yaw', str(LIDAR_YAW),
            '--frame-id', 'base_link', '--child-frame-id', 'base_laser',
        ],
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_imu_link',
        arguments=[
            '--x', str(IMU_X), '--y', str(IMU_Y), '--z', str(IMU_Z),
            '--yaw', str(IMU_YAW),
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
        ],
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', CONFIG_DIR,
            '-configuration_basename', 'cartographer_2d.lua',
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': MAP_RESOLUTION, 'publish_period_sec': 1.0}],
    )

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             ['/maps/', map_name, f'_{stamp}_bag'],
             '/scan', '/tf_static'],
        output='screen',
        condition=IfCondition(record),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='map'),
        DeclareLaunchArgument('record', default_value='true'),
        lidar, tf_lidar, tf_imu, cartographer, occupancy_grid, bag,
    ])
