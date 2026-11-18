#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走行記録（logs/run_*.csv）を読んで、ハンドルの動きをグラフにし、うまく曲がれなかった原因の候補を挙げる。

使い方（ラズパイでもノートパソコンでも）:
    python3 tools/log_kaiseki.py                    # logs/ の一番新しい走行
    python3 tools/log_kaiseki.py logs/run_20261118_111500.csv
    python3 tools/log_kaiseki.py 走行のzip            # get_log.py でダウンロードした zip

結果: 画面に診断、同じフォルダに「〜_kaiseki.png」（グラフ）を保存する。
ファイルを読むだけで、設定は書き換えない。
"""
import csv
import glob
import io
import json
import os
import sys
import zipfile


def yomikomi(path):
    """csv と、同じ名前の _settei.json を読む。zip のときは中の一番新しい走行"""
    if path.endswith(".zip"):
        z = zipfile.ZipFile(path)
        csvs = sorted(n for n in z.namelist() if n.endswith(".csv"))
        if not csvs:
            raise SystemExit("zip の中に csv がありません")
        name = csvs[-1]
        rows = list(csv.reader(io.TextIOWrapper(z.open(name), encoding="utf-8")))
        settei = {}
        s_name = name[:-4] + "_settei.json"
        if s_name in z.namelist():
            settei = json.load(z.open(s_name))
        base = os.path.join(os.path.dirname(path) or ".", os.path.basename(name)[:-4])
        return rows, settei, base
    with open(path, encoding="utf-8") as f:
        rows = list(csv.reader(f))
    settei = {}
    if os.path.exists(path[:-4] + "_settei.json"):
        with open(path[:-4] + "_settei.json", encoding="utf-8") as f:
            settei = json.load(f)
    return rows, settei, path[:-4]


def kazu(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        kouho = sorted(glob.glob("logs/run_*.csv"))
        if not kouho:
            raise SystemExit("logs/ に走行記録がありません。ファイルを指定してください")
        path = kouho[-1]
    rows, settei, base = yomikomi(path)
    if len(rows) < 3:
        raise SystemExit("記録が短すぎます（走り出す前に止まった可能性）")
    head, data = rows[0], rows[1:]
    col = {h: i for i, h in enumerate(head)}

    def retsu(name):
        i = col.get(name)
        return [kazu(r[i]) if i is not None and i < len(r) else None for r in data]

    t = retsu("時間[秒]")
    steer = retsu("ハンドル[%]")
    zure = retsu("横ズレ[px]")
    accel = retsu("アクセル[%]")
    line = retsu("ライン")
    pd = retsu("今の方式のハンドル[%]")
    pp = retsu("PPのハンドル[%]")
    mx = retsu("PP目標の前[m]")
    ten = retsu("PP点の数")
    hiro = retsu("ラインの広がり")
    hoji = retsu("ハンドル保持")
    hoshiki = [r[col["方式"]] if "方式" in col and col["方式"] < len(r) else "" for r in data]
    aru_pp = "PPのハンドル[%]" in col

    N = len(data)
    jikan = (t[-1] or 0) - (t[0] or 0)
    haba = float(settei.get("KIREKAKU_HABA", 1.0) or 1.0)
    print(f"走行記録: {os.path.basename(base)}   {N} コマ・{jikan:.1f} 秒"
          f"（約 {N / max(jikan, 0.01):.0f} コマ/秒）")
    print(f"設定: 方式 LINE_ALGO={settei.get('LINE_ALGO', '（記録なし＝ピュアパシュートを入れる前の版）')}"
          f"  ACCEL_SAIDAI={settei.get('ACCEL_SAIDAI')}  P={settei.get('STEER_P_GAIN')}"
          f"  D={settei.get('STEER_D_GAIN')}  KIREKAKU_DO={settei.get('KIREKAKU_DO')}"
          f"  先読み={settei.get('PP_LD_KIHON')}m  キャリブレーション={'済み' if settei.get('PP_CALIB_ZUMI') else 'まだ'}")

    shindan = []
    # 1. どちらの方式で走ったか
    n_pp = sum(1 for h in hoshiki if h == "pp")
    if settei.get("LINE_ALGO") == "pp":
        wari = n_pp / N
        print(f"\nピュアパシュートで走ったコマ: {n_pp}/{N}（{wari:.0%}）。残りは目標点が決まらず今の方式で走った")
        if wari < 0.7:
            shindan.append(f"ピュアパシュートで走れたのは {wari:.0%} だけです。目標点が決まらないコマが多く、"
                           "その間は今の方式で走っています。キャリブレーション（pp_test.py の真上から見た図）と、"
                           "ラインのしきい値（白黒で見る）を確かめてください")
    # 2. ラインを見失ったコマ
    n_lost = sum(1 for v in line if v == 0)
    if n_lost:
        print(f"ラインが近くの帯に無かったコマ: {n_lost}/{N}（{n_lost / N:.0%}）")
        if n_lost / N > 0.1:
            shindan.append("ラインが近くの帯から外れたコマが多いです。カーブで車がラインから大きく外れたか、"
                           "ラインが画面の端から出ています")
    # 3. ハンドルが左右いっぱいに張り付いていないか
    hari = [i for i, v in enumerate(steer) if v is not None and abs(v) >= 0.97 * haba]
    if hari:
        renzoku, saidai, mae = 0, 0, -2
        for i in hari:
            renzoku = renzoku + 1 if i == mae + 1 else 1
            saidai = max(saidai, renzoku)
            mae = i
        print(f"ハンドルが左右いっぱい（±{haba:.2f}）のコマ: {len(hari)}/{N}、最長 {saidai} コマ続けて")
        if saidai >= 8:
            shindan.append("ハンドルが左右いっぱいのまま曲がりきれていません。プログラムは目いっぱい切ろうとしているので、"
                           "原因は車の側です：①カーブが車の最小回転半径よりきつい ②KIREKAKU_HABA（servo_test の切れ幅）が"
                           "小さい ③速すぎてタイヤが滑っている（アクセルを下げて試す）")
    else:
        print(f"ハンドルが左右いっぱい（±{haba:.2f}）になったコマ: 0")
    m = max((abs(v) for v in steer if v is not None), default=0)
    print(f"ハンドルの最大: {m:.2f}（左右いっぱい ±{haba:.2f} に対して {m / haba:.0%}）")
    if not hari and m < 0.6 * haba and n_lost / N > 0.05:
        shindan.append(f"ハンドルを最大でも {m / haba:.0%} しか切らないうちにラインから外れています。"
                       "切る量が足りません。今の方式なら STEER_P_GAIN を上げる、ピュアパシュートなら "
                       "KIREKAKU_DO を実測値に合わせる（実際より大きい値だと切り足りない）か、先読み距離を短くする")
    # 3b. 今の方式の「ハンドル保持」（ラインが横に広く写るとハンドルを止める安全装置）
    hv = [i for i, v in enumerate(hoji) if v == 1 and line[i] == 1]
    if "ハンドル保持" in col:
        hs = settei.get("STEER_HOJI_HIROGARI", 0.35)
        print(f"今の方式がハンドルを保持したコマ（ラインは見えている）: {len(hv)}/{N}"
              f"（ラインの広がりが {hs} 以上でハンドルを止める）")
        pd_de = [i for i in hv if hoshiki[i] != "pp"]
        if len(pd_de) >= 5:
            shindan.append(f"今の方式で走っている間に、ハンドルの保持が {len(pd_de)} コマ働いています。"
                           "カーブでラインが斜めに広く写り、安全装置がハンドルを直前の値で止めて、切り増せなかった"
                           f"可能性があります。STEER_HOJI_HIROGARI（いま {hs}）を 0.5 くらいに上げて試すか、"
                           "ピュアパシュートで走らせてください（ピュアパシュートにはこの安全装置がありません）")
    # 4. 2つの方式の向きが合っているか
    pares = [(a, b) for a, b in zip(pd, pp) if a is not None and b is not None and (abs(a) > 0.05 or abs(b) > 0.05)]
    if aru_pp and len(pares) >= 10:
        onaji = sum(1 for a, b in pares if a * b > 0) / len(pares)
        print(f"今の方式とピュアパシュートのハンドルの向きが同じコマ: {onaji:.0%}")
        if onaji < 0.5:
            shindan.append("2つの方式でハンドルの向きが逆のコマが多いです。キャリブレーションの左右が逆か、"
                           "カメラの左右反転（CAM_HFLIP）の設定が食い違っている可能性があります")
        pd_m = max(abs(a) for a, _ in pares)
        pp_m = max(abs(b) for _, b in pares)
        print(f"  ハンドルの最大  今の方式 {pd_m:.2f} / ピュアパシュート {pp_m:.2f}")
    # 5. ピュアパシュートの点の数・目標点の距離
    tv = [v for v in ten if v is not None]
    mv = [v for v in mx if v is not None]
    if tv:
        print(f"PP点の数: 平均 {sum(tv) / len(tv):.0f}・最小 {min(tv):.0f}"
              + (f"   目標点の前方距離: 平均 {sum(mv) / len(mv):.2f}m・最小 {min(mv):.2f}m" if mv else ""))
        if mv and min(mv) < 0.25:
            shindan.append("目標点がとても近くになったコマがあります。カーブでラインが画面の横から出て、"
                           "遠くまで見えていない可能性があります（カメラを上向き・広角に、または速度を下げる）")
    if not aru_pp:
        shindan.append("この記録にはピュアパシュートの列がありません（入れる前の版の rc_race.py で走った記録）")

    print("\n■ 原因の候補")
    for s in shindan or ["記録からは、はっきりした原因が見つかりませんでした。グラフと走行動画（.mp4）を見比べてください"]:
        print("  ・" + s)

    # --- グラフ ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for f in ("Noto Sans CJK JP", "IPAexGothic", "IPAGothic", "TakaoGothic", "Yu Gothic", "Meiryo", "MS Gothic"):
            if any(f in x.name for x in matplotlib.font_manager.fontManager.ttflist):
                plt.rcParams["font.family"] = f
                break
        tt = [v if v is not None else float("nan") for v in t]
        f_, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        nan = lambda xs: [v if v is not None else float("nan") for v in xs]
        ax[0].plot(tt, nan(steer), color="black", lw=2, label="実際に出したハンドル")
        if aru_pp:
            ax[0].plot(tt, nan(pd), color="#d95b5b", lw=1, alpha=.8, label="今の方式の計算")
            ax[0].plot(tt, nan(pp), color="#2f8f8f", lw=1, alpha=.8, label="ピュアパシュートの計算")
            for i, h in enumerate(hoshiki):
                if h == "pp" and tt[i] == tt[i]:
                    ax[0].axvspan(tt[i] - 0.017, tt[i] + 0.017, color="#2f8f8f", alpha=.08, lw=0)
        ax[0].axhline(haba, color="gray", ls="--", lw=.8)
        ax[0].axhline(-haba, color="gray", ls="--", lw=.8)
        ax[0].set_ylabel("ハンドル [%]（+が左）")
        ax[0].legend(loc="upper right", fontsize=9)
        ax[0].set_title("ハンドル（緑の帯＝ピュアパシュートで走ったコマ、点線＝左右いっぱい）")
        ax[1].plot(tt, nan(zure), color="#3b6fb0", label="近くの帯のズレ [px]")
        for i, v in enumerate(line):
            if v == 0 and tt[i] == tt[i]:
                ax[1].axvspan(tt[i] - 0.017, tt[i] + 0.017, color="red", alpha=.15, lw=0)
        ax[1].axhline(0, color="gray", lw=.8)
        ax[1].set_ylabel("ズレ [px]（+が右）")
        ax[1].set_title("ラインのズレ（赤い帯＝ラインが近くの帯に無い）")
        ax[2].plot(tt, nan(accel), color="#b06a2f", label="アクセル [%]")
        if tv:
            a2 = ax[2].twinx()
            a2.plot(tt, nan(ten), color="#2f8f8f", alpha=.6, label="PP点の数")
            a2.set_ylabel("PP点の数")
        ax[2].set_ylabel("アクセル [%]")
        ax[2].set_xlabel("時間 [秒]")
        ax[2].set_title("アクセルと、ピュアパシュートが見つけた点の数")
        plt.tight_layout()
        out = base + "_kaiseki.png"
        plt.savefig(out, dpi=110)
        print(f"\nグラフを保存しました: {out}")
    except ImportError:
        print("\n（matplotlib が無いのでグラフは作りませんでした。pip3 install matplotlib で入ります）")


if __name__ == "__main__":
    main()
