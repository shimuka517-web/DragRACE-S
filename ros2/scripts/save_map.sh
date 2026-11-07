#!/bin/bash
# 地図を保存する（部屋の中で、地図作りの起動ファイルを動かしたまま使う）
#
# 使い方：bash /workspace/ros2/scripts/save_map.sh room
#   → /maps/room.png と /maps/room.yaml ができる
#      （ラズパイ本体では ~/DragRACE-S/maps/ に入る）

set -e
NAME="${1:-map}"

echo "1/2 地図の仕上げの最適化をしています…"
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" > /dev/null
sleep 3

echo "2/2 地図を保存しています…"
ros2 run nav2_map_server map_saver_cli -f "/maps/${NAME}" --fmt png \
  --ros-args -p map_subscribe_transient_local:=false

echo "保存しました：/maps/${NAME}.png と /maps/${NAME}.yaml"
