# ROS 2 部屋（Docker）の使い方

ラズパイ4の上で、ROS 2 Jazzy の環境を Docker コンテナ（部屋）として動かします。
ラズパイ本体の OS と DragRACE はそのまま使えます。

## 初回だけ：部屋を作る

```bash
cd ~/DragRACE-S/docker
docker compose build
```

数十分かかることがあります。

## 部屋を起動する

```bash
cd ~/DragRACE-S/docker
docker compose up -d
```

## 部屋の中に入る

```bash
docker compose exec ros bash
```

何度でも入れます（ターミナルを複数開いて、それぞれで入るのも可）。
出るときは `exit`。

## 部屋を止める

```bash
docker compose down
```

## 部屋の中の場所

| 部屋の中 | ラズパイ本体 | 内容 |
|---|---|---|
| /workspace | ~/DragRACE-S | リポジトリ全体 |
| /maps | ~/DragRACE-S/maps | 地図の保存先（Git には含めない） |
| /ros2_ws | （部屋の中だけ） | LiDAR ドライバー |

## 動作確認

部屋の中で：

```bash
ros2 pkg list | grep -E "ldlidar|slam_toolbox|cartographer"
```

3種類のパッケージ名が表示されれば準備完了です。
