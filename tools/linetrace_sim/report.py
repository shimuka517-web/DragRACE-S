import json, numpy as np
R=json.load(open('results.json'))
N={"PD0":"現行PD(初期値)","PD":"現行PD(調整後)","PP":"ピュアパシュート","MPP":"複数点PP","ST":"スタンレー","FF":"曲率先出し","MPC":"簡易モデル予測"}
tracks=list(R["detail"]["PP"].keys())
print("best",R["best"])
print("\n== 完走できた最高速度 (3回とも完走) ==")
for k in N:
    row=[]
    for t in tracks:
        vmax=0
        for v,rs in R["detail"][k][t].items():
            if all(r["ok"] for r in rs): vmax=max(vmax,float(v))
        row.append(vmax)
    print(f"{N[k]:14s}",row)
for v in ("1.5","2.5"):
    print(f"\n== {v} m/s: RMSずれ[cm] / 最大ずれ[cm] / ハンドルの忙しさ[rad/s] ==")
    for k in N:
        row=[]
        for t in tracks:
            rs=R["detail"][k][t][v]
            if all(r["ok"] for r in rs):
                row.append("%4.1f/%4.1f/%4.2f"%(100*np.mean([r["rms"] for r in rs]),100*np.max([r["max"] for r in rs]),np.mean([r["steer"] for r in rs])))
            else: row.append("  失敗(%d/3)  "%sum(not r["ok"] for r in rs))
        print(f"{N[k]:14s}",row)
print("\n== 条件を悪くしたとき 2.0m/s: 完走数/3, 最大ずれcm ==")
for lab,d in R["robust"].items():
    print(lab)
    for k in N:
        row=[]
        for t in tracks:
            rs=d[k][t]; ok=[r for r in rs if r["ok"]]
            row.append("%d/3 %s"%(len(ok), "%4.1f"%(100*max(r["max"] for r in ok)) if ok else "  - "))
        print(f"  {N[k]:14s}",row)
print("\n== 見える距離 (画面上端 f_top) : 完走数/12, 平均ハンドル忙しさ ==")
for ft,d in R["view"].items():
    for k in d:
        for v,rs in d[k].items():
            ok=[r for r in rs if r["ok"]]
            print(ft,N[k],v,"m/s  %d/12"%len(ok),"steer %.2f"%np.mean([r["steer"] for r in ok]) if ok else "", "max %.1f"%(100*max(r["max"] for r in ok)) if ok else "")
print("\n== 速度計画: 平均速度[m/s] (完走数/3) ==")
for key,d in R["plan"].items():
    print("vmax/横G",key)
    for k in d:
        row=[]
        for t in tracks:
            rs=d[k][t]; ok=[r for r in rs if r["ok"]]
            row.append("%.2f(%d)"%(np.mean([r["vavg"] for r in ok]),len(ok)) if ok else " - (0)")
        print(f"  {N[k]:14s}",row)
