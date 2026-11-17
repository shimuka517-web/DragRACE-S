#!/usr/bin/env python3
"""各方式の調整（グリッド探索）と評価。結果を results.json に保存する"""
import itertools, json, sys
from multiprocessing import Pool
import numpy as np
import sim

TR = sim.tracks()

def make(kind, p):
    if kind == "PD": return sim.PDpx(*p, name="現行PD(調整後)")
    if kind == "PP": return sim.PurePursuit(*p, name="ピュアパシュート")
    if kind == "MPP": return sim.MultiPP(p[0], p[1], name="複数点ピュアパシュート")
    if kind == "ST": return sim.Stanley(*p)
    if kind == "FF": return sim.PreviewFF(*p)
    if kind == "MPC": return sim.Rollout(*p)

GRID = {
    "PD": list(itertools.product([0.003, 0.005, 0.007, 0.010, 0.014], [0.0, 0.02, 0.035, 0.06, 0.10], [0.0, 0.3, 0.6, 1.0])),
    "PP": list(itertools.product([0.35, 0.45, 0.55, 0.65, 0.75, 0.90], [0.0, 0.1, 0.2], [0.0, 0.3, 0.6])),
    "MPP": list(itertools.product([0.35, 0.45, 0.55, 0.65, 0.75, 0.90], [0.0, 0.1, 0.2])),
    "ST": list(itertools.product([0.5, 1.0, 2.0, 4.0], [0.3, 1.0], [0.5, 0.8, 1.0])),
    "FF": list(itertools.product([0.4, 0.6, 0.8], [0.5, 1.0, 2.0], [0.3, 0.6, 1.0], [0.5, 1.0])),
    "MPC": list(itertools.product([0.25, 0.35, 0.50], [0.1, 0.3, 1.0], [0.0, 1.0, 3.0])),
}

def score(args):
    kind, p = args
    J = 0.0
    for ti in range(len(TR)):
        for v in (1.5, 2.5):
            r = sim.run(TR[ti], make(kind, p), v, seed=11)
            if not r["ok"]: J += 10.0
            else: J += max(0.0, r["max"] - 0.15) / 0.05 + r["steer"] / 2.0 + r["rms"] / 0.06
    return J

def evaluate(args):
    kind, p, ti, v, seed, noise, delay, f_top, plan = args
    sim.DELAY_FRAMES = delay
    c = sim.PDpx() if kind == "PD0" else make(kind, p)
    return sim.run(TR[ti], c, v, seed=seed, noise=noise, f_top=f_top, speed_plan=plan)

if __name__ == "__main__":
    out = {"best": {}, "speed": {}, "detail": {}, "robust": {}, "view": {}, "plan": {}}
    with Pool() as pool:
        best = {"PD0": (0.007, 0.035, 0.0)}
        for kind, grid in GRID.items():
            js = pool.map(score, [(kind, p) for p in grid])
            k = int(np.argmin(js)); best[kind] = grid[k]
            print(kind, "best", grid[k], "J=%.2f" % js[k], flush=True)
        out["best"] = {k: list(v) for k, v in best.items()}
        kinds = ["PD0", "PD", "PP", "MPP", "ST", "FF", "MPC"]
        speeds = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        seeds = [1, 2, 3]
        jobs = [(k, best[k], ti, v, s, 1.0, 2, None, None) for k in kinds for ti in range(len(TR)) for v in speeds for s in seeds]
        res = pool.map(evaluate, jobs)
        for j, r in zip(jobs, res):
            out["detail"].setdefault(j[0], {}).setdefault(TR[j[2]].name, {}).setdefault(str(j[3]), []).append(r)
        # 条件を悪くしたとき（ノイズ2倍 / 遅れ3フレーム）、2.0 m/s
        for label, noise, delay in (("ノイズ2倍", 2.0, 2), ("遅れ3フレーム", 1.0, 3), ("ノイズ2倍＋遅れ3", 2.0, 3)):
            jobs = [(k, best[k], ti, 2.0, s, noise, delay, None, None) for k in kinds for ti in range(len(TR)) for s in seeds]
            res = pool.map(evaluate, jobs)
            for j, r in zip(jobs, res):
                out["robust"].setdefault(label, {}).setdefault(j[0], {}).setdefault(TR[j[2]].name, []).append(r)
        # カメラがどこまで遠くを見るか（画面の上端の位置）による違い
        for f_top in (0.55, 0.40, 0.30, 0.20, 0.15):
            jobs = [(k, best[k], ti, v, s, 1.0, 2, f_top, None) for k in ("PP", "MPC") for ti in range(len(TR)) for v in (2.0, 3.0) for s in seeds]
            res = pool.map(evaluate, jobs)
            for j, r in zip(jobs, res):
                out["view"].setdefault(str(f_top), {}).setdefault(j[0], {}).setdefault(str(j[3]), []).append(r)
        # カーブの手前で減速する（速度計画）
        for vmax in (3.0, 4.0, 5.0):
            for ay in (3.0, 4.5, 6.0):
                jobs = [(k, best[k], ti, vmax, s, 1.0, 2, None, (vmax, ay)) for k in ("PD", "PP", "MPP", "MPC") for ti in range(len(TR)) for s in seeds]
                res = pool.map(evaluate, jobs)
                for j, r in zip(jobs, res):
                    out["plan"].setdefault(f"{vmax}/{ay}", {}).setdefault(j[0], {}).setdefault(TR[j[2]].name, []).append(r)
    json.dump(out, open("results.json", "w"), ensure_ascii=False, indent=1)
    print("saved")
