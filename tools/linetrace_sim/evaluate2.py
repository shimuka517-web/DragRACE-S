#!/usr/bin/env python3
"""第2ラウンド: 輪郭たどり方式の検出（ラインが横向きになっても点を取る）での再調整と評価"""
import itertools, json
from multiprocessing import Pool
import numpy as np
import sim

TR = sim.tracks()
def make(kind, p):
    return sim.PurePursuit(*p) if kind == "PP" else sim.MultiPP(p[0], p[1])

def score(a):
    kind, p = a; J = 0.0
    for ti in range(len(TR)):
        for v in (1.5, 2.5):
            for seed in (11, 12):
                r = sim.run(TR[ti], make(kind, p), v, seed=seed, det="skel")
                J += 10.0 if not r["ok"] else max(0.0, r["max"] - 0.15) / 0.05 + r["steer"] / 2.0 + r["rms"] / 0.06
    return J

def ev(a):
    kind, p, ti, v, seed, noise, delay, f_top, plan, mode = a
    sim.DELAY_FRAMES = delay
    return sim.run(TR[ti], make(kind, p), v, seed=seed, noise=noise, f_top=f_top, speed_plan=plan, det="skel", plan_mode=mode)

if __name__ == "__main__":
    out = {"best": {}, "detail": {}, "robust": {}, "view": {}, "plan": {}}
    G = {"PP": list(itertools.product([0.30, 0.35, 0.40, 0.45, 0.55, 0.65], [0.0, 0.1, 0.15, 0.2, 0.25], [0.0, 0.3])),
         "MPP": list(itertools.product([0.30, 0.35, 0.40, 0.45, 0.55, 0.65], [0.0, 0.1, 0.15, 0.2, 0.25]))}
    seeds = [1, 2, 3, 4, 5]
    with Pool() as pool:
        best = {}
        for k, g in G.items():
            js = pool.map(score, [(k, p) for p in g]); i = int(np.argmin(js)); best[k] = g[i]
            order = np.argsort(js)[:5]
            print(k, "best", g[i], "J=%.2f" % js[i], "上位:", [(g[j], round(js[j], 1)) for j in order], flush=True)
        out["best"] = {k: list(v) for k, v in best.items()}
        speeds = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        jobs = [(k, best[k], ti, v, s, 1.0, 2, None, None, "line") for k in G for ti in range(len(TR)) for v in speeds for s in seeds]
        for j, r in zip(jobs, pool.map(ev, jobs)):
            out["detail"].setdefault(j[0], {}).setdefault(TR[j[2]].name, {}).setdefault(str(j[3]), []).append(r)
        for label, noise, delay in (("ノイズ2倍", 2.0, 2), ("遅れ3フレーム", 1.0, 3), ("ノイズ2倍＋遅れ3", 2.0, 3)):
            jobs = [(k, best[k], ti, 2.0, s, noise, delay, None, None, "line") for k in G for ti in range(len(TR)) for s in seeds]
            for j, r in zip(jobs, pool.map(ev, jobs)):
                out["robust"].setdefault(label, {}).setdefault(j[0], {}).setdefault(TR[j[2]].name, []).append(r)
        for f_top in (0.55, 0.40, 0.30, 0.20, 0.15):
            jobs = [("PP", best["PP"], ti, v, s, 1.0, 2, f_top, None, "line") for ti in range(len(TR)) for v in (1.5, 2.0, 2.5, 3.0) for s in seeds]
            for j, r in zip(jobs, pool.map(ev, jobs)):
                out["view"].setdefault(str(f_top), {}).setdefault(str(j[3]), []).append(r)
        for mode in ("line", "arc"):
            for vmax in (3.0, 4.0, 5.0):
                for ay in (3.0, 4.5, 6.0):
                    jobs = [("PP", best["PP"], ti, vmax, s, 1.0, 2, None, (vmax, ay), mode) for ti in range(len(TR)) for s in seeds]
                    for j, r in zip(jobs, pool.map(ev, jobs)):
                        out["plan"].setdefault(mode, {}).setdefault(f"{vmax}/{ay}", {}).setdefault(TR[j[2]].name, []).append(r)
    json.dump(out, open("results2.json", "w"), ensure_ascii=False, indent=1)

    R = out; tracks = [t.name for t in TR]
    print("\n完走できた最高速度(5回とも):")
    for k in G:
        print(k, [max([float(v) for v, rs in R["detail"][k][t].items() if all(r["ok"] for r in rs)] or [0]) for t in tracks])
    for v in ("1.5", "2.5"):
        print(v, "m/s RMS/最大/忙しさ")
        for k in G:
            row = []
            for t in tracks:
                rs = R["detail"][k][t][v]
                row.append("%4.1f/%4.1f/%4.2f" % (100 * np.mean([r["rms"] for r in rs]), 100 * np.max([r["max"] for r in rs]), np.mean([r["steer"] for r in rs])) if all(r["ok"] for r in rs) else "失敗%d/5" % sum(not r["ok"] for r in rs))
            print(" ", k, row)
    for lab, d in R["robust"].items():
        for k in G:
            print(lab, k, ["%d/5 %s" % (sum(r["ok"] for r in d[k][t]), "%.1f" % (100 * max([r["max"] for r in d[k][t] if r["ok"]] or [0]))) for t in tracks])
    print("見える距離:")
    for ft, d in R["view"].items():
        print(ft, "(%.2fm)" % (sim.row_to_dist(float(ft)) + sim.CAM_X), {v: "%d/20" % sum(r["ok"] for r in rs) for v, rs in d.items()})
    print("速度計画 平均速度(完走数/5):")
    for mode, dd in R["plan"].items():
        for key, d in dd.items():
            print(mode, key, ["%.2f(%d)" % (np.mean([r["vavg"] for r in d[t] if r["ok"]] or [0]), sum(r["ok"] for r in d[t])) for t in tracks])
