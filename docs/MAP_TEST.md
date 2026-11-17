# map_test.py 設計書（地図作り専用プログラム）

2026-11-17 決定。画面イメージは `docs/images/map_test_ui.png`。

## 目的

servo_test.py・throttle_test.py と同じ使い方で、地図作りだけを行う専用プログラム。
ゆっくりライントレースで 1 周しながら Cartographer で地図を作り、コース名を付けて保存する。
保存したコースの中から「参照するコース」を選ぶ（rc_race.py が位置推定と画面表示に使う）。

## 使い方

```bash
cd ~/DragRACE-S
python3 map_test.py
```

ブラウザで `http://<車のIPアドレス>:8080`。
rc_race.py と同じ 8080 番を使うので、同時には起動できない。

## 全体の構成

| 場所 | 役割 |
|---|---|
| map_test.py（ラズパイ本体、Docker の外） | 画面、カメラ、ライントレース、モーター、IMU、Docker の操作 |
| Docker の部屋 `dragrace-s-ros` | LD19、Cartographer、橋渡しノード、生データの記録 |
| `maps/live/`（共有フォルダ） | 部屋 → map_test.py へのデータの受け渡し |

ラズパイ本体には ROS 2 を入れない。部屋とのやり取りは「docker コマンド」と「maps/live/ のファイル」だけ。

## rc_race.py から流用するもの

`import rc_race` で読み込む（rc_race.py は `if __name__ == "__main__"` で守られているので、読み込んでも走り出さない。
読み込んだ時点で kyotsu_settei.json と kuruma_settei.json の値も反映される）。

| 流用するもの | 用途 |
|---|---|
| `rc_race.Kamera` | カメラ画像の取得 |
| `rc_race.me_de_miru(gazou, ato)` | ライン検出（zure・nerai・hirogari・line_mieru） |
| `rc_race.gamen_ni_kaku(...)` | 画面用のカメラ映像（検出の帯や点を描いたもの） |
| `rc_race.TeAshi` | ハンドルとアクセルの出力（`hashiru(steer_sa, accel, kick)`、`tomaru()`、`neutral()`、`close()`） |
| ハンドルの計算式 | rc_race.py の `nou_de_handan` の P+D 制御と同じ式を map_test.py に書く（STEER_P_GAIN、STEER_D_GAIN、SAKIYOMI、HANDORU_GYAKU、KIREKAKU_HABA、STEER_HOJI_HIROGARI をそのまま使う） |

ゴール判定・シグナル発進・アクセルワークは使わない（地図作りでは不要）。
rc_race.py 自体は書き換えない。

## ゆっくりライントレース

- アクセル ＝ 速度スライダーの値（0〜100%）× `rc_race.ACCEL_SAIDAI` × 上限割合
  - 上限割合 `MAP_ACCEL_JOUGEN = 0.40`（設定で変えられる）。スライダー 100% でも ACCEL_SAIDAI の 40% まで
  - 初期値はスライダー 25%
- 発進のときだけ `kick=True`（`rc_race.KICK_BYO` 秒）。L 点を学習した車だけ効く
- 走行中もスライダーを動かせる

## 安全装置

| 条件 | 動作 |
|---|---|
| servo_test・throttle_test が済んでいない（`TeAshi.chosei_zumi` が False） | 走行開始ボタンを押せない（rc_race.py の ARM と同じ） |
| 「地図作り 開始」を押していない | 走行開始ボタンを押せない |
| ラインを見失った（`rc_race.ROSUTO_KAKUTEI` フレーム続いた） | 停止（ブレーキ→N）。地図作りは続ける |
| ブラウザとの通信が `rc_race.HEARTBEAT_BYO` 秒途切れた | 停止 |
| 走行時間が `MAP_SAIDAI_SOKO_BYO = 180` 秒を超えた | 停止 |
| LiDAR のデータが 2 秒届かない | 停止して画面に警告 |
| 緊急停止ボタン・Ctrl+C | すぐ停止 |
| 中止・保存・プログラム終了 | 走行中なら先に停止 |

ESC の電源は map_test.py を起動したあとに入れる（既存のプログラムと同じ注意を画面に表示）。

## 地図作りの操作（Docker の操作）

部屋のコンテナ名は `dragrace-s-ros`、起動ファイルは `/workspace/ros2/launch/mapping.launch.py`。
コマンドの前に `source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash` が要る。

| ボタン | 中でやること |
|---|---|
| （起動時） | 部屋が止まっていたら `docker compose -f docker/compose.yaml up -d` |
| 地図作り 開始 | ①残っているドライバー等を止める（`pkill -f ldlidar` など）②`docker exec -d dragrace-s-ros bash -c '… ros2 launch …mapping.launch.py map_name:=tmp_<日時>'` ③`maps/live/` を空にする |
| この名前で保存 | ①走行中なら停止 ②`docker exec dragrace-s-ros bash -c '… bash /workspace/ros2/scripts/save_map.sh <名前>'` ③生データのフォルダ `tmp_<日時>_bag` を `<名前>_bag` に名前変更 ④地図作りを止める |
| 中止（捨てる） | 走行停止 → 地図作りを止める → 生データのフォルダを消すか確認 |

- コース名に使える文字：半角英数字と `_` `-` だけ（ファイル名になるため）
- 同じ名前のコースがあるときは上書きの確認を出す

## 橋渡しノード（部屋の中、新規）

`ros2/scripts/map_bridge.py`（rclpy）。mapping.launch.py から一緒に起動する。
1 秒ごとに次の 3 つを `/maps/live/` に書き出す（一時ファイルに書いてから名前変更し、読みかけを防ぐ）。

| ファイル | 中身 |
|---|---|
| `map.json` | `/map`（OccupancyGrid）の width・height・resolution・origin と、マスの値（-1/0〜100）を base64 にしたもの |
| `pose.json` | `map → base_link` の x・y・ヨー角、時刻 |
| `status.json` | /scan の 1 秒あたりの回数、1 周で使える点の数、最後に受け取った時刻 |

部屋に新しいパッケージを入れなくて済むよう、画像への変換はしない（画像は map_test.py 側で numpy と cv2 で作る）。
このしくみは、あとで rc_race.py の画面に現在位置を出すときにもそのまま使う。

## 画面（docs/images/map_test_ui.png）

左：作成中の地図（白＝走れる所、黒＝壁、灰色＝未測定）、現在位置と向き（オレンジの矢印）、走った軌跡（青）、
スタート地点（緑）、1m の目盛り。その下に走った距離・地図の大きさ・速さ・ずれ補正の状態。
センサーとラズパイの状態（LiDAR の回数・使える点・IMU のヨーの回転・CPU 温度）。

右：地図作り（①開始 ②ゆっくり1周 ③保存 の段階表示、開始・中止ボタン、コース名と保存ボタン）、
ゆっくりライントレース（カメラ映像、速度スライダー、走行開始・停止、緊急停止）、
保存済みのコース（一覧、参照中の印、「参照するコースにする」「見る」ボタン）。

- 走った軌跡は `pose.json` を map_test.py がためていく（地図作りを開始するたびにリセット）
- IMU は map_test.py が I2C で直接読む（tools/imu_test.py と同じ方法）
- CPU 温度は `/sys/class/thermal/thermal_zone0/temp`。70℃ で黄色、80℃ で赤
- 画面の更新は 1 秒ごと（地図と状態）、カメラ映像は rc_race.HAISHIN_FPS

## 参照するコース

- `kuruma_settei.json` に `"COURSE_NAME": "<コース名>"` を書く（説明の行も付ける）
- rc_race.py の位置推定と画面表示は、この名前の `maps/<名前>.png・.yaml・.pbstream` を使う（rc_race.py 側は別の作業）
- 保存済みのコースの一覧は `maps/*.yaml` から作る（保存日時はファイルの日付、大きさは yaml の解像度と画像の大きさから計算）

## 設定ファイルに足す値（kuruma_settei_sample.json に説明付きで追加）

| 値 | 初期値 | 意味 |
|---|---|---|
| `MAP_ACCEL_JOUGEN` | 0.40 | 地図作り中のアクセルの上限（ACCEL_SAIDAI に対する割合） |
| `MAP_SAIDAI_SOKO_BYO` | 180 | 地図作り中の最大走行時間[秒] |
| `COURSE_NAME` | "" | 参照するコース名 |

## 作る順番（それぞれ動作確認してから次へ）

1. 橋渡しノード（map_bridge.py）と mapping.launch.py への追加 → `maps/live/` に 3 つのファイルが出るか
2. map_test.py の画面と地図表示（走行なし、車は手で押す）→ 地図・現在位置・軌跡が出るか
3. 地図作りの開始・保存・中止、保存済みのコースの一覧と切り替え
4. ゆっくりライントレースと安全装置（**最初は車輪を浮かせて確認**）
5. 実際にコースを 1 周して地図を作る
