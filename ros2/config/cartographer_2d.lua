-- DragRACE-S 用 Cartographer 設定（2D、LiDAR のみ、車輪の回転情報なし）
-- 起動ファイル：ros2/launch/mapping.launch.py
-- 「--」の後ろは説明です。値を変えたら、起動し直すだけで反映されます。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",        -- 車の基準点（後輪の車軸の中心の真下）
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,           -- 車輪の回転情報がないので、Cartographer が odom を作る
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,                -- エンコーダーを付けたら true にする
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                 -- /scan を 1 本使う
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2        -- ラズパイ4 の 4 コアのうち、裏の最適化に使う数

-- ===== LiDAR の点の絞り込み =====
-- 最小距離 [m]：これより近い点を捨てる。車体やケーブルを拾うなら大きくする
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- 最大距離 [m]：これより遠い点を捨てる。ダクトのすき間から会場の壁を拾うなら小さくする
--   LD19 の仕様は 12m だが、遠いほど誤差が増えるので少し短めにしている
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- 測れなかった方向を「この距離まで何もない」として扱う長さ [m]
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
-- 間引き [m]：この大きさのマスごとに点を 1 つにまとめる。ラズパイが重いときは大きくする
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
-- IMU は今は使わない（IMU のノードを作ったら true にする）
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 1 回の位置合わせに使うスキャンの数（LD19 は 1 周で 1 回分）
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ===== スキャン同士の位置合わせ =====
-- 車輪の回転情報がないので、毎回スキャンの形から移動量を探す（少し重いが必須）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 1 回のスキャンの間に動ける距離 [m] と角度。速く押す・回すとはみ出して地図がずれる
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- ほとんど動いていないときは地図に足さない（ラズパイの負担を減らす）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- 小さな地図（サブマップ）1 枚に入れるスキャンの数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- ===== ループ閉じ込み（スタート地点に戻ってきたときのずれ補正）=====
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
