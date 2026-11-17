# 地図作りの手順（Cartographer）

LD19 だけで地図を作ります。車輪の回転情報や IMU はまだ使いません。
コマンドはすべてラズパイで打ちます。

## 準備：部屋を起動して中に入る

```bash
cd ~/DragRACE-S/docker
docker compose up -d
docker compose exec ros bash
```

先頭が `root@` になれば部屋の中です。

## 1. 地図作りを始める

```bash
pkill -f ldlidar
ros2 launch /workspace/ros2/launch/mapping.launch.py map_name:=room > /tmp/mapping.log 2>&1 &
sleep 8
```

`room` の部分は地図の名前です。会場ごとに変えてください。

## 2. ちゃんと動いているか確認する

```bash
ros2 topic hz /map
```

1 秒に 1 回くらい表示が出れば、地図が作られています。Ctrl+C で止めます。
何も出ないときは `tail -50 /tmp/mapping.log` で中身を確認します。

## 3. 車をゆっくり動かす

- ゆっくりのライントレースで 1 周させる（rc_race.py は Docker の外で、いつもどおり起動する）
- 手で押してもよい
- 歩くくらいの速さより遅く、急に曲げない
- できればスタート地点に戻ってくる（戻ってきたときにずれが補正される）

## 4. 地図を保存する

```bash
bash /workspace/ros2/scripts/save_map.sh room
```

ラズパイ本体の `~/DragRACE-S/maps/` に 3 つのファイルができます。

- `room.png`・`room.yaml`：地図の画像と設定（画面表示に使う）
- `room.pbstream`：Cartographer の地図データ（走行中の位置推定に使う）

この 3 つをまとめて 1 つのコースとして扱います。コースを作り直すときは別の名前で保存し、
どのコースを使うかは設定画面のコース名で切り替えます（予定）。
LiDAR の生データも `room_日時_bag` というフォルダに保存されています。

## 5. 止める

```bash
kill %1
pkill -f ldlidar
```

## 地図をノートパソコンで見る

ノートパソコンの Git Bash で（`ラズパイの宛先` は普段 ssh で使っているもの）：

```bash
scp ラズパイの宛先:~/DragRACE-S/maps/room.png ~/Downloads/
```

## 地図が歪んだときに調整する場所

`ros2/config/cartographer_2d.lua` に説明付きでまとめてあります。

- 曲がると壁が二重になる → もっとゆっくり回す。`angular_search_window` を大きくする
- 車体を拾っている → `min_range` を大きくする、または起動ファイルの無視する角度を広げる
- 会場の外の壁が映る → `max_range` を小さくする
- ラズパイが重い → `voxel_filter_size` を大きくする
