#!/usr/bin/env python3
"""カメラ画像を合成して、rc_race.py のライン検出と line_algo.py（ピュアパシュート）を
実物のプログラムのまま走らせる試験。リポジトリの一番上のフォルダで実行する:

    python3 tools/linetrace_sim/gazou_test.py

ラインの色は rc_race.py の LINE_IRO（黒線 / 白線）に合わせて描く。

確かめること:
  1. キャリブレーション（四角形の4つの角 → 画素⇔床の変換）の精度
  2. 合成画像でコースを1周できるか（今の方式とピュアパシュートを同じ条件で比べる）
  3. 1コマあたりの計算時間
"""
import math, os, sys, time
import numpy as np
import cv2

sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim
import line_algo
import rc_race

W, H = rc_race.CAM_WIDTH, rc_race.CAM_HEIGHT
FX = (W / 2) / math.tan(sim.HFOV_HALF)

def yuka_kara_gaso(pitch):
    """床(x,y,1) → 画素 の行列（カメラの高さ・向きから作った『本当の』変換）"""
    c, s = math.cos(pitch), math.sin(pitch)
    f = np.array([c, 0, -s]); r = np.array([0, -1, 0]); d = np.array([-s, 0, -c])
    t = np.array([sim.CAM_X, 0, sim.CAM_H])
    # p = (x, y, 0) - t → [Xc, Yc, Zc] = [p·r, p·d, p·f]
    R = np.vstack([r, d, f])
    M = np.column_stack([R[:, 0], R[:, 1], -R @ t])
    K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
    return K @ M

G0 = yuka_kara_gaso(sim.CAM_PITCH)

def satsuei(trk, i0, x, y, yaw, rng, sen_haba=0.03, noise=1.0):
    """車の位置から見たカメラ画像を作る（灰色の床に黒いライン）"""
    S = 300                                   # 真上から見た下絵の細かさ [画素/m]
    idx = (np.arange(i0 - 40, i0 + 420, 2)) % trk.n
    dx, dy = trk.xy[idx, 0] - x, trk.xy[idx, 1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    xv, yv = c * dx + s * dy, -s * dx + c * dy
    # ラインの色は rc_race.py の LINE_IRO に合わせる（黒線: 明るい床に黒 / 白線: 暗い床に白）
    yuka_iro, sen_iro = (170, 30) if rc_race.LINE_IRO == "kuro" else (110, 235)
    shitae = np.full((int(3.2 * S), int(3.0 * S)), yuka_iro, np.uint8)     # x: -0.2〜3.0, y: ±1.5
    pts = np.column_stack([(1.5 - yv) * S, (3.0 - xv) * S]).astype(np.int32)
    cv2.polylines(shitae, [pts.reshape(-1, 1, 2)], False, sen_iro, max(1, int(round(sen_haba * S))), cv2.LINE_AA)
    G = yuka_kara_gaso(sim.CAM_PITCH + rng.normal(0, math.radians(0.4)) * noise)   # 縦揺れ
    A = np.array([[0, -1 / S, 3.0], [-1 / S, 0, 1.5], [0, 0, 1]])                 # 下絵の画素 → 床
    img = cv2.warpPerspective(shitae, G @ A, (W, H), flags=cv2.INTER_LINEAR, borderValue=yuka_iro)
    img = np.clip(img.astype(np.float32) + rng.normal(0, 6 * noise, img.shape), 0, 255).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

def calib(rng, zure_px=1.0):
    """四角形（横40cm×奥行き60cm、手前の辺が車軸から45cm）の角をクリックしたことにして変換を作る"""
    yuka = np.array([[0.45, 0.2], [0.45, -0.2], [1.05, -0.2], [1.05, 0.2]])
    q = np.column_stack([yuka, np.ones(4)]) @ G0.T
    px = q[:, :2] / q[:, 2:3] + rng.normal(0, zure_px, (4, 2))
    px = px[rng.permutation(4)]                               # クリックの順番はばらばら
    hk, riyu = line_algo.Henkan.shikaku_kara(px, 0.40, 0.60, 0.45, 0.0, (W, H))
    assert hk is not None, riyu
    return hk

def hashiru(trk, hoshiki, v, seed=1, sen_haba=0.03, noise=1.0, kiroku=None):
    rng = np.random.default_rng(seed)
    hk = calib(rng)
    p = dict(line_algo.SHOKICHI, PP_HENKAN=hk.lista(), PP_HENKAN_SIZE=[W, H], PP_SOKUDO_KEISU=1.0)
    pp = line_algo.PurePursuit(p)
    i, x, y, yaw = 0, trk.xy[0, 0], trk.xy[0, 1], trk.yaw[0]
    delta, cmd_q = 0.0, [0.0] * sim.DELAY_FRAMES
    dt = 1.0 / sim.CTRL_HZ / sim.SIM_SUB
    ato, zure_zenkai, sa_mae, lost = None, None, 0.0, 0
    errs, rates, jikan, dist, prev = [], [], [], 0.0, 0.0
    y_chikaku = H * (rc_race.ROI_CHIKAKU_UE + rc_race.ROI_CHIKAKU_SHITA) / 2
    while dist < trk.len:
        i, e = trk.nearest(x, y, i); errs.append(e)
        if e > 0.45: return dict(ok=False, why="コースアウト", dist=dist)
        img = satsuei(trk, i, x, y, yaw, rng, sen_haba, noise)
        t0 = time.perf_counter()
        kekka = rc_race.me_de_miru(img, ato)
        if kekka["line_mieru"]: ato = kekka["zure"]
        # 今の方式（rc_race.py の nou_de_handan と同じ式）
        sa_pd = sa_mae
        if kekka["zure"] is not None and kekka["hirogari"] < rc_race.STEER_HOJI_HIROGARI:
            sabun = 0.0 if zure_zenkai is None else kekka["zure"] - zure_zenkai
            zure_zenkai = kekka["zure"]
            sa_pd = max(-1.0, min(1.0, -(rc_race.STEER_P_GAIN * kekka["nerai"] + rc_race.STEER_D_GAIN * sabun)))
        else:
            zure_zenkai = None
        mieru = kekka["line_mieru"]
        sa = sa_pd
        if hoshiki == "pp":
            sen = line_algo.sen_dake_tsukuru(img, rc_race.LINE_IRO, rc_race.KURO_SHIKII, rc_race.SHIRO_SHIKII)
            r = pp.keisan(sen, kekka["zure"], y_chikaku, accel=v)
            if r["ok"]: sa, mieru = r["sa"], True
            if kiroku is not None and len(kiroku) < 400: kiroku.append((img, r, hk))
        jikan.append(time.perf_counter() - t0)
        sa_mae = sa
        lost = 0 if mieru else lost + 1
        if lost >= rc_race.ROSUTO_KAKUTEI: return dict(ok=False, why="ライン見失い", dist=dist)
        cmd = sa * sim.DELTA_MAX
        rates.append((cmd - prev) * sim.CTRL_HZ); prev = cmd
        cmd_q.append(cmd); applied = cmd_q.pop(0)
        for _ in range(sim.SIM_SUB):
            delta += float(np.clip((applied - delta) / sim.STEER_TAU * dt, -sim.STEER_RATE * dt, sim.STEER_RATE * dt))
            kap = math.tan(delta) / sim.L
            klim = sim.A_LAT_MAX / max(v * v, 1e-6)
            kap = max(-klim, min(klim, kap))
            x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw += v * kap * dt; dist += v * dt
    errs = np.array(errs)
    return dict(ok=True, rms=float(np.sqrt(np.mean(errs ** 2))), max=float(errs.max()),
                steer=float(np.sqrt(np.mean(np.square(rates)))), ms=1000 * float(np.mean(jikan)))

if __name__ == "__main__":
    rng = np.random.default_rng(0)
    # 1. キャリブレーションの精度
    gosa = []
    for _ in range(200):
        hk = calib(rng)
        yuka = np.array([[0.3, 0.0], [0.6, 0.15], [0.9, -0.2], [1.2, 0.3]])
        q = np.column_stack([yuka, np.ones(4)]) @ G0.T
        gosa.append(np.hypot(*(hk.px_kara_yuka(q[:, :2] / q[:, 2:3]) - yuka).T))
    g = np.array(gosa) * 100
    print("キャリブレーションの誤差[cm]（クリックが1画素ずれる場合）  距離0.3m: %.1f  0.6m: %.1f  0.9m: %.1f  1.2m: %.1f"
          % tuple(np.percentile(g, 90, axis=0)))
    print("見える距離:", [None if v is None else round(v, 2) for v in calib(rng).mieru_kyori()])
    # 2. 合成画像で1周
    for trk in sim.tracks():
        for v in (1.0, 1.5, 2.0, 2.5):
            gyo = []
            for hoshiki in ("pd", "pp"):
                rs = [hashiru(trk, hoshiki, v, seed=s) for s in (1, 2)]
                ok = [r for r in rs if r["ok"]]
                gyo.append("%s %d/2 %s" % (hoshiki, len(ok),
                           "ずれ%4.1f/%4.1fcm 忙しさ%4.2f %4.1fms" % (100 * np.mean([r["rms"] for r in ok]),
                           100 * max(r["max"] for r in ok), np.mean([r["steer"] for r in ok]), np.mean([r["ms"] for r in ok]))
                           if ok else rs[0]["why"]))
            print(f"{trk.name:10s} {v} m/s | " + " | ".join(gyo), flush=True)
