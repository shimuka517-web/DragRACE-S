# DragRACE-S 用 ROS 2 環境（ラズパイ4 / arm64）
# ROS 2 Jazzy をベースに、LiDAR ドライバーと地図作りツールを入れた部屋を作る

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 地図作り・地図保存・センサー確認に使うパッケージ
#   slam_toolbox      : 地図作りツールの候補1
#   cartographer_ros  : 地図作りツールの候補2（LiDAR と IMU だけで動く）
#   nav2_map_server   : 作った地図をファイルに保存・読み込みする
#   i2c-tools, smbus  : 部屋の中から IMU（I2C）を確認・読み取りする
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cartographer-ros \
    ros-jazzy-nav2-map-server \
    i2c-tools \
    python3-smbus \
    && rm -rf /var/lib/apt/lists/*

# LD19 のドライバー（LDROBOT 公式の ldlidar_stl_ros2）をソースからビルド
WORKDIR /ros2_ws/src
RUN git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
# 新しいコンパイラでは pthread.h が自動で読み込まれずビルドに失敗するため、先頭に追加する
RUN sed -i '1i #include <pthread.h>' ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# 部屋に入ったら毎回 ROS 2 とドライバーを使える状態にする
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
