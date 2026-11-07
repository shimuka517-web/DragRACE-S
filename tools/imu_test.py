#!/usr/bin/env python3
"""LSM6DSV16X 動作確認スクリプト（ラズパイ4 / I2C）

やること
  1. センサーの名札（WHO_AM_I）を読んで、LSM6DSV16X か確認する
  2. 起動直後の 2 秒間、止まった状態でジャイロのずれ（バイアス）を測る
  3. 加速度・ジャイロ・ヨー角を 0.1 秒ごとに表示する

使い方
  python3 tools/imu_test.py
  止めるときは Ctrl+C

軸の向きの調べ方
  - 平らに置いたとき、加速度で約 +9.8 が出る軸が「上向き」の軸（たぶん Z）
  - 片側を持ち上げると、持ち上げた向きに応じて X か Y の値が変わる
  - 平らなまま机の上で回すと gz とヨー角が動く
"""

import math
import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

# ---- 設定 ----
I2C_BUS = 1
ADDR = 0x6B            # i2cdetect で見えたアドレス（0x6A の基板もある）
CALIB_SEC = 2.0        # ジャイロのずれを測る時間（この間は動かさない）
PRINT_INTERVAL = 0.1   # 表示の間隔 [秒]

# ---- レジスタ（データシートより）----
WHO_AM_I = 0x0F        # 読むと 0x70 が返る
CTRL1 = 0x10           # 加速度の更新速度
CTRL2 = 0x11           # ジャイロの更新速度
CTRL3 = 0x12           # データ読み出しの設定
CTRL6 = 0x15           # ジャイロの測定範囲
CTRL8 = 0x17           # 加速度の測定範囲
OUT_TEMP_L = 0x20      # ここから 温度2 + ジャイロ6 + 加速度6 バイト

ODR_120HZ = 0x06       # 1 秒に 120 回更新
GYRO_FS_1000DPS = 0x03 # ±1000 °/s
ACC_FS_4G = 0x01       # ±4 g

GYRO_SENS = 35.0e-3    # ±1000 dps のとき 1 カウント = 0.035 °/s
ACC_SENS = 0.122e-3    # ±4 g のとき 1 カウント = 0.122 mg
G = 9.80665


def to_int16(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v >= 32768 else v


def setup(bus):
    who = bus.read_byte_data(ADDR, WHO_AM_I)
    if who != 0x70:
        raise SystemExit(f"WHO_AM_I が 0x{who:02X} でした（0x70 のはず）。配線とアドレスを確認してください")
    print("LSM6DSV16X を確認しました (WHO_AM_I = 0x70)")

    # BDU=1（上位と下位のバイトを同じタイミングの値にそろえる）、IF_INC=1（連続読み出し）
    bus.write_byte_data(ADDR, CTRL3, 0x44)
    bus.write_byte_data(ADDR, CTRL6, GYRO_FS_1000DPS)
    bus.write_byte_data(ADDR, CTRL8, ACC_FS_4G)
    bus.write_byte_data(ADDR, CTRL1, ODR_120HZ)   # 加速度：高性能モード 120Hz
    bus.write_byte_data(ADDR, CTRL2, ODR_120HZ)   # ジャイロ：高性能モード 120Hz
    time.sleep(0.1)


def read(bus):
    d = bus.read_i2c_block_data(ADDR, OUT_TEMP_L, 14)
    temp = to_int16(d[0], d[1]) / 256.0 + 25.0
    gx, gy, gz = (to_int16(d[i], d[i + 1]) * GYRO_SENS for i in (2, 4, 6))
    ax, ay, az = (to_int16(d[i], d[i + 1]) * ACC_SENS * G for i in (8, 10, 12))
    return (ax, ay, az), (gx, gy, gz), temp


def main():
    with SMBus(I2C_BUS) as bus:
        setup(bus)

        print(f"{CALIB_SEC:.0f} 秒間ジャイロのずれを測ります。動かさないでください…")
        n, sx, sy, sz = 0, 0.0, 0.0, 0.0
        t_end = time.time() + CALIB_SEC
        while time.time() < t_end:
            _, (gx, gy, gz), _ = read(bus)
            sx, sy, sz, n = sx + gx, sy + gy, sz + gz, n + 1
            time.sleep(0.005)
        bias = (sx / n, sy / n, sz / n)
        print(f"ずれ: gx={bias[0]:+.3f} gy={bias[1]:+.3f} gz={bias[2]:+.3f} [°/s]  ({n} 回平均)")
        print("Ctrl+C で終了\n")

        yaw = 0.0
        t_prev = time.time()
        t_print = t_prev
        while True:
            (ax, ay, az), (gx, gy, gz), temp = read(bus)
            gx, gy, gz = gx - bias[0], gy - bias[1], gz - bias[2]

            now = time.time()
            yaw += gz * (now - t_prev)        # 回転の速さを積み重ねてヨー角にする
            t_prev = now
            yaw = (yaw + 180.0) % 360.0 - 180.0

            if now - t_print >= PRINT_INTERVAL:
                t_print = now
                print(
                    f"加速度[m/s²] x={ax:+6.2f} y={ay:+6.2f} z={az:+6.2f} | "
                    f"ジャイロ[°/s] x={gx:+7.1f} y={gy:+7.1f} z={gz:+7.1f} | "
                    f"ヨー角 {yaw:+7.1f}° | {temp:4.1f}℃"
                )
            time.sleep(0.005)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n終了しました")
