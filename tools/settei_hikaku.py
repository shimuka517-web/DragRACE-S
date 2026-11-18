#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""授業用 DragRACE と DragRACE-S の、車ごとの設定（kuruma_settei.json）を比べる。

kuruma_settei.json は Git に入らないので、フォルダごとに別のファイルがある。
同じ車なのに片方だけ走らない、という時は、まずこれで違いを確かめる。
ファイルを読むだけで、書き換えはしない。

使い方（ラズパイで）:
    cd ~/DragRACE-S
    python3 tools/settei_hikaku.py              # ~/DragRACE と比べる
    python3 tools/settei_hikaku.py ../別のフォルダ
"""
import json
import os
import sys

# rc_race.py の初期値（設定ファイルに書いていない値はこれになる）
SHOKICHI = {"SUROTTORU_GYAKU": False, "ZENSHIN_SAITEI": None, "ZENKAI_HABA": 2.75,
            "ACCEL_SAIDAI": 5, "ACCEL_HASSHIN": 20, "ACCEL_RAMPU": 20, "KICK_BYO": 0.3,
            "BRAKE": 50, "BRAKE_BYO": 1.0, "ACCEL_KAGEN": 0,
            "ACCEL_KIRIMASHI_GAIN": 0, "ACCEL_KIRIMODOSHI_GAIN": 0, "PP_CURVE_GEN": 0.0}
GAKUSHU = ["STEER_CHURITSU", "THROTTLE_N", "ZENSHIN_KYOKAI", "KOTAI_KYOKAI", "ZENSHIN_SAITEI"]


def yomu(folder):
    v = dict(SHOKICHI)
    for name in ("kyotsu_settei.json", "kuruma_settei.json"):   # rc_race.py と同じ順番
        p = os.path.join(folder, name)
        if os.path.exists(p):
            with open(p, encoding="utf-8") as f:
                v.update({k: x for k, x in json.load(f).items() if not k.startswith("#")})
    return v


def keisan(v):
    """rc_race.py の TeAshi と同じ計算で、走る時・止める時の信号[%]を出す"""
    n, d, r, L = (v.get(k) for k in ("THROTTLE_N", "ZENSHIN_KYOKAI", "KOTAI_KYOKAI", "ZENSHIN_SAITEI"))
    if None in (n, d, r, v.get("STEER_CHURITSU")):
        return None, ["学習値がそろっていません（servo_test・throttle_test が必要）。rc_race.py は走行開始できません"]
    muki = -1 if v["SUROTTORU_GYAKU"] else 1
    chui = []
    zero = d
    if L is not None:
        if 0 < (L - n) * muki <= (d - n) * muki:
            zero = L
        else:
            chui.append(f"L点 {L} が N({n}) と D点({d}) の間にありません → D点を0%として扱います")
    haba = v["ZENKAI_HABA"]
    saidai = v["ACCEL_SAIDAI"] * (1 - 0.5 * (v["PP_CURVE_GEN"] > 0))
    kick = zero + muki * haba * min(v["ACCEL_SAIDAI"], v["ACCEL_HASSHIN"]) / 100
    kick = max(kick, d) if muki > 0 else min(kick, d)
    hashiru = zero + muki * haba * v["ACCEL_SAIDAI"] / 100
    brake = r - muki * haba * v["BRAKE"] / 100
    s = dict(muki=muki, n=n, d=d, r=r, zero=zero, kick=kick, hashiru=hashiru, brake=brake,
             saitei=zero + muki * haba * saidai / 100)
    # 向きの確かめ: D点は N より前進側、R点は後退側にあるはず
    if (d - n) * muki <= 0 or (r - n) * muki >= 0:
        chui.append("D点・R点と SUROTTORU_GYAKU（スロットルの向き）が食い違っています。"
                    "前進と後退が逆の信号になっている可能性が高いです")
    if v["PP_CURVE_GEN"] > 0:
        chui.append(f"PP_CURVE_GEN = {v['PP_CURVE_GEN']}（カーブでアクセルを緩める）が入っています。"
                    "ピュアパシュートで走ると、アクセルが最大で半分まで下がります")
    if abs(hashiru - n) < abs(d - n):
        chui.append(f"走行中の信号 {hashiru:.2f}% は、タイヤが回り始めるD点 {d:.2f}% より弱いです。"
                    "発進キックのあと止まってしまう可能性があります（ACCEL_SAIDAI を上げる・KICK_BYO を長く）")
    return s, chui


def hyoji(namae, v):
    print(f"\n■ {namae}")
    s, chui = keisan(v)
    if s:
        print(f"  スロットルの向き: {'低い%で前進（TT-02 など）' if s['muki'] < 0 else '高い%で前進（RD2.0 など）'}"
              f"  SUROTTORU_GYAKU={v['SUROTTORU_GYAKU']}")
        print(f"  N(停止) {s['n']:.2f}%   D点(回り始め) {s['d']:.2f}%   R点(後退の回り始め) {s['r']:.2f}%"
              f"   アクセル0% = {s['zero']:.2f}%")
        print(f"  発進キック {s['kick']:.2f}%  →  走行中 {s['hashiru']:.2f}%（ACCEL_SAIDAI {v['ACCEL_SAIDAI']}%）"
              f"  →  STOP時のブレーキ {s['brake']:.2f}%（BRAKE {v['BRAKE']}%、{v['BRAKE_BYO']}秒）")
    for c in chui:
        print("  ⚠ " + c)
    return s


def main():
    koko = os.getcwd()
    aite = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(koko, "..", "DragRACE"))
    for f in (koko, aite):
        if not os.path.exists(os.path.join(f, "kuruma_settei.json")):
            print(f"⚠ {f} に kuruma_settei.json がありません")
    a, b = yomu(aite), yomu(koko)
    print(f"比べるフォルダ:  A = {aite}\n                 B = {koko}（いまのフォルダ）")

    print("\n■ 値が違うもの（A → B）。PP_ で始まるピュアパシュートの値は除く")
    kazu = 0
    for k in sorted(set(a) | set(b)):
        if k.startswith("PP_") or k in ("LINE_ALGO",):
            continue
        if a.get(k) != b.get(k):
            kazu += 1
            shirushi = "★" if k in GAKUSHU or k in SHOKICHI else "  "
            print(f"  {shirushi} {k}: {a.get(k)} → {b.get(k)}")
    if kazu == 0:
        print("  （違いはありません）")
    print("  ★ = 走り出し・ブレーキに関係する値")

    sa = hyoji("A: " + os.path.basename(aite), a)
    sb = hyoji("B: " + os.path.basename(koko), b)
    if sa and sb and (abs(sa["kick"] - sb["kick"]) > 0.005 or abs(sa["hashiru"] - sb["hashiru"]) > 0.005):
        print("\n→ 発進・走行中の信号が A と B で違います。A で走れて B で走れないなら、これが原因です。")
        print("  A の値を B に合わせるか、B のフォルダで throttle_test.py をやり直してください。")
    print("\n参考: STOP を押すとブレーキとして後退側の信号を出します。車が止まっている時は、"
          "ESC がこれを『後退』として扱うので、思いきり下がります。")


if __name__ == "__main__":
    main()
