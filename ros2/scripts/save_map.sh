#!/bin/bash
# 地図を保存する（部屋の中で、地図作りの起動ファイルを動かしたまま使う）
#
# 使い方：bash /workspace/ros2/scripts/save_map.sh room
#   → /maps/room.png・room.yaml・room.pbstream の 3 つができる
#      （ラズパイ本体では ~/DragRACE-S/maps/ に入る）
#
#   room.png / room.yaml : 地図の画像と設定（画面表示に使う）
#   room.pbstream        : Cartographer の地図データ（走行中の位置推定に使う）

set -e
NAME="${1:-map}"

echo "1/3 地図の仕上げの最適化をしています…"
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" > /dev/null
sleep 3

echo "2/3 位置推定用の地図データを保存しています…"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/maps/${NAME}.pbstream', include_unfinished_submaps: true}" > /dev/null

echo "3/3 地図の画像を保存しています…"
ros2 run nav2_map_server map_saver_cli -f "/maps/${NAME}" --fmt png \
  --ros-args -p map_subscribe_transient_local:=false

echo "保存しました：/maps/${NAME}.png・${NAME}.yaml・${NAME}.pbstream"
