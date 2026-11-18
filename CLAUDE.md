# DragRACE-S（Claude Code 向けの説明書）

カメラでライントレースする 1/10 RC カー（授業用 DragRACE をもとにした個人開発用）。
目標は MAGP 出場。LiDAR で地図を作り、自己位置推定してコース幅を使った速い経路を走らせる。
計画の全体は `docs/PLAN.md`、地図作りの手順は `docs/MAPPING.md`。

## ユーザーについて

- 説明は日本語で。Git・Linux・ROS 2 はまだ初心者なので、何をするかを短く説明してから進める
- 図より文章での説明を好む

## 2 台のマシン

| マシン | 役割 | 場所 |
|---|---|---|
| ノートパソコン（Windows、ここ） | 編集・git commit・push | `~/DragRACE-S` |
| ラズパイ4（Raspberry Pi OS 64bit / Debian 13、4GB） | 実行（カメラ・センサー・モーター） | `ssh pi099@099` の `~/DragRACE-S` |

- 流れ：ノートパソコンで編集 → commit → push → `ssh pi099@099 "cd ~/DragRACE-S && git pull"` → ラズパイで実行
- ラズパイ上でファイルを直接編集しない（変更はノートパソコンから git で届ける）
- 作業ブランチは `feature/ros2`。機能ごとにブランチを分ける方針
- origin = github.com/shimuka517-web/DragRACE-S（公開）、upstream = RC-DragRACE/DragRACE（本家、取り込みは重視しない）

## ROS 2（Docker の部屋）

ROS 2 Jazzy はラズパイ上の Docker コンテナ `dragrace-s-ros` で動く。設定は `docker/`。
コンテナ内では `/workspace` = リポジトリ、`/maps` = `~/DragRACE-S/maps`（Git に含めない）。

```bash
# 起動
ssh pi099@099 "cd ~/DragRACE-S/docker && docker compose up -d"

# 中でコマンドを 1 回実行（ROS 2 の環境を読み込んでから）
ssh pi099@099 "cd ~/DragRACE-S/docker && docker compose exec -T ros bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list'"

# 動かしっぱなしにする処理（ドライバーや地図作り）は -d で裏で起動し、ログは /tmp に出す
ssh pi099@099 "cd ~/DragRACE-S/docker && docker compose exec -d ros bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch /workspace/ros2/launch/mapping.launch.py map_name:=room > /tmp/mapping.log 2>&1'"

# 停止
ssh pi099@099 "cd ~/DragRACE-S/docker && docker compose down"
```

- Dockerfile を変えたら `docker compose build`（数分〜数十分）
- カメラ（リボンケーブル）は Docker の外の DragRACE で扱う

## センサー（詳細は docs/PLAN.md）

- IMU：LSM6DSV16X、I2C バス 1、アドレス 0x6B。確認は `python3 tools/imu_test.py`（ラズパイ本体で、Docker の外）
  - センサーの −Y が車の前、+X が左、+Z が上。ヨーは反時計回りがプラス
- LiDAR：LD19、`/dev/ttyUSB0`、/scan 約 10Hz、0° ≒ 車の前、反時計回り
  - 後ろ 150°〜210° は車体が映るので無視（起動ファイルで設定済み）
  - **ドライバーが 2 つ動くとスキャンが乱れる。起動前に必ず `pkill -f ldlidar`**
- 取り付け位置（base_link ＝ 後輪車軸の中心の真下の床）：LD19 (0.225, 0, 0.115)、IMU (0.170, 0, 0.037, ヨー +90°)

## ライントレースの方式

- `LINE_ALGO`（kuruma_settei.json）で切り替える。`pd`＝今の方式、`pp`＝ピュアパシュート（line_algo.py）
- `pp_test.py`：キャリブレーションと確認の画面。**モーターもハンドルも動かさない**
- 使い方は `docs/PP_TEST.md`、方式の比較は `docs/LINETRACE_EVAL.md`
- 授業用 DragRACE と走り方が違うときは `python3 tools/settei_hikaku.py`（ラズパイで。~/DragRACE と kuruma_settei.json を比べ、出る信号を計算する。読むだけ）
- rc_race.py・line_algo.py を直したら `python3 tools/linetrace_sim/gazou_test.py`（合成画像での試験、数分）で壊れていないか確かめる。ノートパソコンでも動く（numpy と opencv-python が要る）

## 必ずユーザーに確認してから行うこと

- **モーター・スロットル・ステアリングを動かすもの**（rc_race.py、throttle_test.py、servo_test.py など）。車が走り出して危ないので、実行前に車輪が浮いているか・周りが安全かを必ず確認する
- `sudo` を使うもの、ラズパイの再起動・シャットダウン
- ファイルやフォルダの削除、`git push --force`、main ブランチへの push や merge
- 時間のかかる `docker compose build`

## 確認なしで行ってよいこと

- ファイルの読み取り、git status / log / diff
- feature ブランチでの commit と push
- ラズパイでの git pull、センサーの読み取りだけのコマンド（/scan の確認、imu_test.py、pp_test.py など）
