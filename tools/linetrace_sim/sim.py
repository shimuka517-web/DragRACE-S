#!/usr/bin/env python3
"""ライントレース方式の比較シミュレーション（DragRACE-S）

車（自転車モデル）・カメラの見え方（行ごとのライン検出、ノイズ、遅れ）を再現し、
複数のハンドル制御方式を同じ条件で走らせて比べる。
使い方: python3 evaluate.py   （結果は results.json と標準出力）
"""
import math
import numpy as np

# ---------------- 車 ----------------
L = 0.257                 # ホイールベース [m]
DELTA_MAX = math.radians(25)   # 最大舵角
STEER_TAU = 0.04          # サーボの遅れ [s]
STEER_RATE = 4.5          # 舵角の最大速度 [rad/s]
A_LAT_MAX = 0.8 * 9.81    # タイヤの横グリップ限界 [m/s^2]
CTRL_HZ = 30              # カメラ・制御の周期
SIM_SUB = 4               # 1 制御周期あたりの計算回数
DELAY_FRAMES = 2          # 撮影からハンドルが動き出すまでの遅れ [フレーム]

# ---------------- カメラ（仮定。実車で測り直す）----------------
CAM_H = 0.25              # カメラの高さ [m]
CAM_PITCH = math.radians(30)   # 下向きの角度
CAM_X = 0.10              # 後輪車軸からカメラまでの前後距離 [m]
VFOV_HALF = math.radians(24.4)
HFOV_HALF = math.radians(31.0)
IMG_W, IMG_H = 640, 480
ROW_STEP = 4              # 何行おきに調べるか（今の TADORI_KIZAMI と同じ）

def row_to_dist(f):
    """画面の縦位置 f（0=上端, 1=下端）→ カメラから地面の点までの前方距離 [m]"""
    a = np.arctan((f - 0.5) * 2 * math.tan(VFOV_HALF))
    return CAM_H / np.tan(CAM_PITCH + a)

def half_width(d):
    """前方距離 d での、画面の半分の横幅 [m]"""
    return np.sqrt(d * d + CAM_H * CAM_H) * math.tan(HFOV_HALF)

# ---------------- コース（ラインの形）----------------
def _resample(xy, ds=0.01):
    xy = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = int(s[-1] / ds)
    si = np.linspace(0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])])

def _smooth_closed(xy, win):
    k = np.ones(win) / win
    out = np.empty_like(xy)
    for i in range(2):
        ext = np.concatenate([xy[-win:, i], xy[:, i], xy[:win, i]])
        out[:, i] = np.convolve(ext, k, mode='same')[win:-win]
    return out

def track_stadium(length=12.0, width=6.0, r=1.5, wig_amp=0.0, wig_len=1.5):
    """長円コース。wig_amp>0 でラインを蛇行させる（MAGP の『最適でないライン』）"""
    pts = []
    sx, sy = length / 2 - r, width / 2 - r
    def arc(cx, cy, a0, a1):
        for a in np.linspace(a0, a1, 60, endpoint=False):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    for x in np.linspace(-sx, sx, 200, endpoint=False): pts.append((x, -width / 2))
    arc(sx, -sy, -math.pi / 2, 0)
    for y in np.linspace(-sy, sy, 100, endpoint=False): pts.append((length / 2, y))
    arc(sx, sy, 0, math.pi / 2)
    for x in np.linspace(sx, -sx, 200, endpoint=False): pts.append((x, width / 2))
    arc(-sx, sy, math.pi / 2, math.pi)
    for y in np.linspace(sy, -sy, 100, endpoint=False): pts.append((-length / 2, y))
    arc(-sx, -sy, math.pi, 1.5 * math.pi)
    xy = _resample(np.array(pts))
    if wig_amp > 0:
        d = np.gradient(xy, axis=0); d /= np.hypot(d[:, 0], d[:, 1])[:, None]
        nrm = np.column_stack([-d[:, 1], d[:, 0]])
        s = np.arange(len(xy)) * 0.01
        n_w = round(s[-1] / wig_len)
        w = wig_amp * np.sin(2 * math.pi * n_w * s / s[-1]) + 0.3 * wig_amp * np.sin(2 * math.pi * (2 * n_w + 1) * s / s[-1] + 1.0)
        xy = _resample(xy + nrm * w[:, None])
    return xy

def track_technical():
    """S字・ヘアピン・シケインのあるテクニカルコース（最小半径 約0.7m）"""
    c = np.array([(0, 0), (3, 0), (5, 0.3), (6.5, 1.6), (6.2, 3.2), (4.6, 3.6), (3.6, 2.6), (2.4, 2.4),
                  (1.6, 3.4), (2.2, 4.8), (4.5, 5.2), (6.5, 5.4), (7.2, 6.6), (6.2, 7.6), (3, 7.6),
                  (0, 7.4), (-1.6, 6.2), (-1.2, 4.6), (-0.2, 3.6), (-1.4, 2.4), (-1.8, 1.0), (-1.0, 0.1)], float)
    xy = _resample(c, 0.01)
    xy = _smooth_closed(xy, 121)
    return _resample(xy)

def track_sharp():
    """テープの角のように、角がきつい（半径 約0.35m）四角いコース"""
    c = np.array([(0, 0), (6, 0), (6, 1.5), (3.5, 1.5), (3.5, 4), (6, 4), (6, 6), (0, 6)], float)
    xy = _resample(c, 0.01)
    xy = _smooth_closed(xy, 45)
    return np.roll(_resample(xy), -250, axis=0)      # スタートを直線の上にする

class Track:
    def __init__(self, xy, name):
        self.xy, self.name, self.n = xy, name, len(xy)
        self.len = self.n * 0.01
        d = np.gradient(np.vstack([xy[-2:], xy, xy[:2]]), axis=0)[2:-2]
        self.yaw = np.arctan2(d[:, 1], d[:, 0])
        dy = np.diff(np.concatenate([self.yaw, self.yaw[:1]])); dy = (dy + math.pi) % (2 * math.pi) - math.pi
        self.kappa = np.convolve(np.concatenate([dy[-10:], dy, dy[:10]]) / 0.01, np.ones(21) / 21, 'same')[10:-10]
    def nearest(self, x, y, i_prev, win=150):
        idx = (np.arange(i_prev - 30, i_prev + win)) % self.n
        d2 = (self.xy[idx, 0] - x) ** 2 + (self.xy[idx, 1] - y) ** 2
        k = int(np.argmin(d2))
        return int(idx[k]), math.sqrt(d2[k])

# ---------------- カメラの見え方 ----------------
class Camera:
    def __init__(self, f_top, rng, noise=1.0, det="row"):
        self.rows_f = np.arange(0.95 * IMG_H, f_top * IMG_H, -ROW_STEP) / IMG_H
        self.rng, self.noise, self.det = rng, noise, det
        self.d_min, self.d_max = float(row_to_dist(0.95)), float(row_to_dist(f_top))
    def see_skel(self, trk, i0, x, y, yaw):
        """輪郭たどり方式: ラインが横向きになっても、つながっている限り点を取る（約3cmおき）"""
        idx = (np.arange(i0 - 20, i0 + 320, 3)) % trk.n
        dx, dy = trk.xy[idx, 0] - x, trk.xy[idx, 1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        xc = c * dx + s * dy - CAM_X
        yc = -s * dx + c * dy
        ok = (xc >= self.d_min) & (xc <= self.d_max)
        ok &= np.abs(yc) <= half_width(np.maximum(xc, 0.01))
        if not ok.any(): return np.zeros((0, 2)), np.zeros(0)
        first = int(np.argmax(ok)); bad = np.flatnonzero(~ok[first:])
        last = first + (int(bad[0]) if len(bad) else len(ok) - first)
        if self.rng.random() < 0.08 * self.noise and last - first > 6:
            last = first + int(self.rng.integers(6, last - first))
        xc, yc = xc[first:last], yc[first:last]
        pitch_err = self.rng.normal(0, math.radians(0.4)) * self.noise
        beta = np.arctan2(CAM_H, xc)                      # 見下ろす角度
        d_meas = CAM_H / np.tan(np.clip(beta + pitch_err, 0.02, 1.5))
        scale = np.sqrt(d_meas ** 2 + CAM_H ** 2) / np.sqrt(xc ** 2 + CAM_H ** 2)
        y_meas = yc * scale + self.rng.normal(0, 2.0 * self.noise, len(xc)) * half_width(xc) / (IMG_W / 2)
        f = 0.5 + np.tan(beta - CAM_PITCH) / (2 * math.tan(VFOV_HALF))
        return np.column_stack([d_meas, y_meas]), f
    def see(self, trk, i0, x, y, yaw):
        if self.det == "skel": return self.see_skel(trk, i0, x, y, yaw)
        """返り値: カメラ座標の点 (前方距離 d, 横位置 y(左が+)) と各点の画面の縦位置 f。見えなければ空"""
        idx = (np.arange(i0 - 20, i0 + 320)) % trk.n
        dx, dy = trk.xy[idx, 0] - x, trk.xy[idx, 1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        xc = c * dx + s * dy - CAM_X
        yc = -s * dx + c * dy
        # 前方距離が増えていく区間だけ（ラインが横向き・手前向きになったらそこで終わり）
        inc = np.diff(xc) > 0.002
        end = len(xc) if inc.all() else int(np.argmin(inc)) + 1
        xc, yc = xc[:end], yc[:end]
        if end < 3: return np.zeros((0, 2)), np.zeros(0)
        # 車体の縦揺れ（ピッチ）で、行と距離の対応がずれる
        pitch_err = self.rng.normal(0, math.radians(0.4)) * self.noise
        a = np.arctan((self.rows_f - 0.5) * 2 * math.tan(VFOV_HALF))
        d_true = CAM_H / np.tan(CAM_PITCH + a + pitch_err)
        ok = (d_true >= xc[0]) & (d_true <= xc[-1]) & (d_true > 0)
        y_hit = np.interp(d_true, xc, yc)
        ok &= np.abs(y_hit) <= half_width(d_true)
        if not ok.any(): return np.zeros((0, 2)), np.zeros(0)
        first = int(np.argmax(ok))
        bad = np.flatnonzero(~ok[first:])
        last = first + (int(bad[0]) if len(bad) else len(ok) - first)
        # ときどき途中までしかたどれない（かすれ・照明）
        if self.rng.random() < 0.08 * self.noise and last - first > 6:
            last = first + int(self.rng.integers(6, last - first))
        f = self.rows_f[first:last]
        d_nom = row_to_dist(f)                       # 制御側が信じている距離（縦揺れは知らない）
        px_noise = self.rng.normal(0, 2.0 * self.noise, last - first)
        y_meas = y_hit[first:last] + px_noise * half_width(d_true[first:last]) / (IMG_W / 2)
        return np.column_stack([d_nom, y_meas]), f

# ---------------- 制御方式 ----------------
class Ctrl:
    name = "base"
    f_top = 0.55            # 画面のどこまで上を見るか
    def reset(self): self.prev = 0.0
    def __call__(self, pts, f, v): raise NotImplementedError

def _to_axle(pts):
    return np.column_stack([pts[:, 0] + CAM_X, pts[:, 1]])

def _mono(p):
    """前方距離が増えていく部分だけを残す（y=f(x) の式あてはめ用）"""
    inc = np.diff(p[:, 0]) > 0
    return p if inc.all() else p[:int(np.argmin(inc)) + 1]

class PDpx(Ctrl):
    """今の rc_race.py と同じ: 近くの帯のズレ[px]に P、ズレの変化に D。SAKIYOMI で遠くの帯を混ぜる"""
    def __init__(self, P=0.007, D=0.035, saki=0.0, name="現行PD"):
        self.P, self.D, self.saki, self.name = P, D, saki, name
    def reset(self): self.prev, self.zprev = 0.0, None
    def __call__(self, pts, f, v):
        near = (f >= 0.78)
        if not near.any():
            self.zprev = None
            return self.prev, False
        px = -pts[:, 1] / half_width(pts[:, 0]) * (IMG_W / 2)     # 画面では右が+
        z_near = px[near].mean()
        far = (f >= 0.55) & (f <= 0.68)
        z_far = px[far].mean() if far.any() else px[-1]
        nerai = (1 - self.saki) * z_near + self.saki * z_far
        sabun = 0.0 if self.zprev is None else z_near - self.zprev
        self.zprev = z_near
        sa = -(self.P * nerai + self.D * sabun)
        self.prev = float(np.clip(sa, -1, 1)) * DELTA_MAX
        return self.prev, True

class PurePursuit(Ctrl):
    """ピュアパシュート: 先読み距離 Ld 先のライン上の点へ、円弧で向かう"""
    def __init__(self, Ld0=0.5, kv=0.0, kc=0.0, f_top=0.15, name="ピュアパシュート"):
        self.Ld0, self.kv, self.kc, self.f_top, self.name = Ld0, kv, kc, f_top, name
    def _target(self, p, Ld):
        r = np.hypot(p[:, 0], p[:, 1])
        if r[-1] <= Ld: return p[-1]
        if r[0] >= Ld: return p[0]
        k = int(np.argmax(r >= Ld)); t = (Ld - r[k - 1]) / (r[k] - r[k - 1] + 1e-9)
        return p[k - 1] + t * (p[k] - p[k - 1])
    def __call__(self, pts, f, v):
        if len(pts) < 3: return self.prev, False
        p = _to_axle(pts)
        Ld = self.Ld0 + self.kv * v
        if self.kc > 0 and len(p) >= 8:
            a = np.polyfit(p[:, 0], p[:, 1], 2)
            Ld = Ld / (1 + self.kc * abs(2 * a[0]))
        Ld = max(Ld, 0.30)
        tx, ty = self._target(p, Ld)
        self.prev = float(np.clip(math.atan(2 * L * ty / (tx * tx + ty * ty)), -DELTA_MAX, DELTA_MAX))
        return self.prev, True

class MultiPP(PurePursuit):
    """複数の先読み距離のピュアパシュートを平均する（ノイズに強くする）"""
    def __call__(self, pts, f, v):
        if len(pts) < 3: return self.prev, False
        p = _to_axle(pts)
        Ld = max(self.Ld0 + self.kv * v, 0.30)
        ks = []
        for m, w in ((0.7, 0.25), (1.0, 0.5), (1.3, 0.25)):
            tx, ty = self._target(p, Ld * m)
            ks.append(w * 2 * ty / (tx * tx + ty * ty))
        self.prev = float(np.clip(math.atan(L * sum(ks)), -DELTA_MAX, DELTA_MAX))
        return self.prev, True

class Stanley(Ctrl):
    """スタンレー: 前輪の位置での『ラインの向きとのずれ』＋『横ずれ』"""
    def __init__(self, k=2.0, ks=0.5, kpsi=1.0, f_top=0.15, name="スタンレー"):
        self.k, self.ks, self.kpsi, self.f_top, self.name = k, ks, kpsi, f_top, name
    def __call__(self, pts, f, v):
        if len(pts) < 6: return self.prev, False
        p = _to_axle(pts); p = p[p[:, 0] < 0.9]
        if len(p) < 6: return self.prev, False
        a = np.polyfit(p[:, 0], p[:, 1], 2)
        e = np.polyval(a, L)                        # 前輪の位置での横ずれ（カメラの死角なので式で延長）
        psi = math.atan(2 * a[0] * L + a[1])
        d = self.kpsi * psi + math.atan(self.k * e / (v + self.ks))
        self.prev = float(np.clip(d, -DELTA_MAX, DELTA_MAX))
        return self.prev, True

class PreviewFF(Ctrl):
    """多項式あてはめ＋曲率の先出し: ラインの曲がり具合ぶんを先に切り、残りのずれを補正"""
    def __init__(self, xla=0.5, ke=1.5, kpsi=0.6, kff=1.0, f_top=0.15, name="曲率先出し＋補正"):
        self.xla, self.ke, self.kpsi, self.kff, self.f_top, self.name = xla, ke, kpsi, kff, f_top, name
    def __call__(self, pts, f, v):
        p = _mono(_to_axle(pts))
        if len(p) < 6: return self.prev, False
        a = np.polyfit(p[:, 0], p[:, 1], 2, w=1.0 / (0.3 + p[:, 0]))
        x = min(self.xla, p[-1, 0])
        e = np.polyval(a, x); psi = math.atan(2 * a[0] * x + a[1])
        kap = 2 * a[0] / (1 + (2 * a[0] * x + a[1]) ** 2) ** 1.5
        d = self.kff * math.atan(L * kap) + self.ke * e + self.kpsi * psi
        self.prev = float(np.clip(d, -DELTA_MAX, DELTA_MAX))
        return self.prev, True

class Rollout(Ctrl):
    """簡易モデル予測: ハンドル角の候補ごとに少し先まで車の動きを計算し、ラインに一番沿うものを選ぶ"""
    def __init__(self, T=0.35, lam=0.3, w_far=1.0, f_top=0.15, name="簡易モデル予測"):
        self.T, self.lam, self.w_far, self.f_top, self.name = T, lam, w_far, f_top, name
        self.cands = np.linspace(-DELTA_MAX, DELTA_MAX, 21)
    def reset(self): self.prev, self.hist = 0.0, [0.0] * DELAY_FRAMES
    def __call__(self, pts, f, v):
        if len(pts) < 8:
            self.hist = self.hist[1:] + [self.prev]
            return self.prev, False
        p = _to_axle(pts)
        a = np.polyfit(p[:, 0], p[:, 1], 3, w=1.0 / (0.3 + p[:, 0]))
        xmax = p[-1, 0]
        dt = 1.0 / CTRL_HZ
        n = max(3, int(self.T / dt))
        C = len(self.cands)
        x = np.zeros(C); y = np.zeros(C); th = np.zeros(C); de = np.full(C, self.prev)
        # 遅れの間は、すでに出した指令で動く
        for cmd in self.hist:
            de = de + np.clip((cmd - de) / STEER_TAU * dt, -STEER_RATE * dt, STEER_RATE * dt) if STEER_TAU > dt else np.full(C, cmd)
            x += v * np.cos(th) * dt; y += v * np.sin(th) * dt; th += v * np.tan(de) / L * dt
        cost = np.zeros(C)
        for i in range(n):
            de = de + np.clip(self.cands - de, -STEER_RATE * dt, STEER_RATE * dt)
            x += v * np.cos(th) * dt; y += v * np.sin(th) * dt; th += v * np.tan(de) / L * dt
            xe = np.minimum(x, xmax)
            w = 1.0 + self.w_far * i / n
            cost += w * (y - np.polyval(a, xe)) ** 2
        cost = cost / n + self.lam * (self.cands - self.prev) ** 2
        k = int(np.argmin(cost))
        # 候補の間を放物線で補間して、きざみを細かくする
        d = self.cands[k]
        if 0 < k < C - 1:
            c0, c1, c2 = cost[k - 1], cost[k], cost[k + 1]
            den = c0 - 2 * c1 + c2
            if den > 1e-12: d += 0.5 * (c0 - c2) / den * (self.cands[1] - self.cands[0])
        self.prev = float(np.clip(d, -DELTA_MAX, DELTA_MAX))
        self.hist = self.hist[1:] + [self.prev]
        return self.prev, True

# ---------------- 走行シミュレーション ----------------
def arc_kappa(pts, r_far=1.1):
    """遠くの点へ向かう円弧の曲がり具合（細かい蛇行に反応しない速度の決定用）"""
    if len(pts) < 3: return None
    p = _to_axle(pts)
    r = np.hypot(p[:, 0], p[:, 1])
    ks = []
    for rr in (0.6, 0.85, r_far):
        k = int(np.argmax(r >= rr)) if r[-1] >= rr else len(r) - 1
        ks.append(abs(2 * p[k, 1] / (r[k] ** 2)))
    return float(max(ks))

def preview_kappa(pts):
    """見えている範囲のラインの曲がり具合（速度の決定用）"""
    p = _mono(_to_axle(pts)) if len(pts) else pts
    if len(p) < 8: return None
    a = np.polyfit(p[:, 0], p[:, 1], 2)
    xs = np.linspace(p[0, 0], p[-1, 0], 8)
    return float(np.max(np.abs(2 * a[0] / (1 + (2 * a[0] * xs + a[1]) ** 2) ** 1.5)))

def run(trk, ctrl, v, seed=0, laps=1.0, noise=1.0, f_top=None, speed_plan=None, a_lat_max=A_LAT_MAX, det="row", plan_mode="line"):
    """1 回走らせて成績を返す。speed_plan=(vmax, 横G目標) を渡すと、カーブの手前で減速する"""
    rng = np.random.default_rng(seed)
    cam = Camera(ctrl.f_top if (f_top is None or isinstance(ctrl, PDpx)) else f_top, rng, noise, "row" if isinstance(ctrl, PDpx) else det)
    ctrl.reset()
    i = 0
    x, y, yaw = trk.xy[0, 0], trk.xy[0, 1], trk.yaw[0]
    delta, cmd_q = 0.0, [0.0] * DELAY_FRAMES
    dt = 1.0 / CTRL_HZ / SIM_SUB
    vv = v if speed_plan is None else min(v, 1.0)
    errs, rates, lost, t, dist, prev_cmd = [], [], 0, 0.0, 0.0, 0.0
    target = trk.len * laps
    max_steps = int(target / max(0.5, vv * 0.3) * CTRL_HZ) + 2000
    for step in range(max_steps):
        i, e = trk.nearest(x, y, i)
        errs.append(e)
        if e > 0.45: return dict(ok=False, why="コースアウト", dist=dist, t=t)
        if rng.random() < 0.02 * noise: pts, f = np.zeros((0, 2)), np.zeros(0)     # 1コマ丸ごと見えない
        else: pts, f = cam.see(trk, i, x, y, yaw)
        cmd, seen = ctrl(pts, f, vv)
        lost = 0 if seen else lost + 1
        if lost >= 10: return dict(ok=False, why="ライン見失い", dist=dist, t=t)
        rates.append((cmd - prev_cmd) * CTRL_HZ); prev_cmd = cmd
        cmd_q.append(cmd); applied = cmd_q.pop(0)
        v_tgt = v
        if speed_plan is not None:
            vmax, ay = speed_plan
            k = preview_kappa(pts) if plan_mode == "line" else arc_kappa(pts)
            v_tgt = vmax if (k is None or k < 1e-3) else min(vmax, math.sqrt(ay / k))
            v_tgt = max(v_tgt, 0.8)
        for _ in range(SIM_SUB):
            delta += float(np.clip((applied - delta) / STEER_TAU * dt, -STEER_RATE * dt, STEER_RATE * dt))
            if speed_plan is not None:
                vv += float(np.clip(v_tgt - vv, -4.0 * dt, 3.0 * dt))
            kap = math.tan(delta) / L
            klim = a_lat_max / max(vv * vv, 1e-6)
            kap = max(-klim, min(klim, kap))        # グリップ限界を超えたぶんは曲がれない（アンダーステア）
            x += vv * math.cos(yaw) * dt; y += vv * math.sin(yaw) * dt; yaw += vv * kap * dt
            dist += vv * dt; t += dt
        if dist >= target: break
    errs, rates = np.array(errs), np.array(rates)
    return dict(ok=True, t=t, dist=dist, rms=float(np.sqrt(np.mean(errs ** 2))), max=float(errs.max()),
                steer=float(np.sqrt(np.mean(rates ** 2))), vavg=dist / t)

def tracks():
    return [Track(track_stadium(wig_amp=0.0), "長円(半径1.5m)"),
            Track(track_stadium(wig_amp=0.10, wig_len=2.0), "長円＋蛇行ライン"),
            Track(track_technical(), "テクニカル"),
            Track(track_sharp(), "きつい角")]
