#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   line_algo.py — ライントレースの新方式「ピュアパシュート」
#
#   rc_race.py の今の方式（近くの帯のズレに P・D 制御）と切り替えて使う部品です。
#   このファイルだけでは動きません。rc_race.py と pp_test.py から呼ばれます。
#   方式を選んだ理由と比較の結果は docs/LINETRACE_EVAL.md を読んでください。
#
# ==============================================================================
#
#  ● 仕組み（人間の運転にたとえると）
#
#      今の方式       … 足元のラインだけを見て、ズレたぶんハンドルを切る
#      ピュアパシュート … 少し先のライン上の1点（目標点）を見て、
#                         そこへ円を描いて向かうようにハンドルを切る
#
#  ● 3つの部品
#
#      1. Henkan … 画面の画素 ⇔ 床の上の位置[m] の変換（pp_test.py でキャリブレーション）
#      2. Tengun … 車を中心にした同心円に沿ってラインを探し、点の列（点群）にする
#      3. PurePursuit … 点群から目標点を決めて、ハンドルの量を計算する
#
#  ● 位置の表し方（このファイルの中はすべてこれ）
#
#      原点 = 後輪の車軸の中心の真下の床。 x = 前[m]、y = 左[m]
#      ハンドルは「左に切る」がプラス
#
# ==============================================================================

import os
import json
import math

import numpy as np
import cv2

SETTEI_FILE = "kuruma_settei.json"

# 設定の初期値（rc_race.py のブロック1 と同じ値。説明もそちらに書いてある）
SHOKICHI = {
    "PP_LD_KIHON": 0.65, "PP_LD_SOKUDO": 0.25, "PP_LD_SAISHO": 0.30,
    "PP_SOKUDO_KEISU": 0.0, "KIREKAKU_DO": 25.0, "WHEELBASE": 0.257,
    "PP_CURVE_GEN": 0.0, "PP_CURVE_SAITEI": 0.5,
    "PP_GATE": 0.25,
    # --- 足元からたどる検出（Kenshutsu）の設定 ---
    "PP_SA": 30,              # 周りの床より、これ以上明るい（黒線なら暗い）所だけをラインとみなす（0〜255）
    "PP_HABA_MIN": 0.020,     # ラインの幅の下限[m]。これより細いものはノイズとして消す
    "PP_HABA_MAX": 0.075,     # ラインの幅の上限[m]。これより太い塊（床の反射など）は消す
    "PP_GATE_Y": 0.30,        # 前のコマのラインの位置から、左右にこの範囲[m]の中だけでラインを探す
    "PP_LD_CURVE": 0.6,       # カーブがきついほど先読み距離を縮める強さ（0＝縮めない）
    "PP_HENKAN": None, "PP_HENKAN_SIZE": None,
}


def settei_toridasu(g):
    """rc_race.py の設定（globals()）から、このファイルで使う値だけを取り出す"""
    return {k: g.get(k, v) for k, v in SHOKICHI.items()}


def settei_kaku(atai):
    """kuruma_settei.json に値を書き足す（ほかの値や説明の行はそのまま残す）"""
    settei = {}
    if os.path.exists(SETTEI_FILE):
        try:
            with open(SETTEI_FILE) as f:
                settei = json.load(f)
        except Exception:
            settei = {}
    settei.update(atai)
    with open(SETTEI_FILE, "w") as f:
        json.dump(settei, f, indent=2, ensure_ascii=False)


KYOTSU_FILE = "kyotsu_settei.json"


def kyotsu_ni_kaku(namae, atai):
    """kyotsu_settei.json（会場で共通の設定）の数値を1つ書き換える。
       このファイルは Git で管理していて、説明の行や空行で読みやすく整えてあるので、
       ファイル全体を書き直さず、その値の数字の所だけを置き換える。
       値がまだ無ければ、最初の「{」のすぐ後ろに足す。"""
    import re
    with open(KYOTSU_FILE, encoding="utf-8") as f:
        moji = f.read()
    kata = re.compile(r'("' + re.escape(namae) + r'"\s*:\s*)(-?\d+(?:\.\d+)?)')
    if kata.search(moji):
        moji = kata.sub(lambda m: m.group(1) + str(atai), moji, count=1)
    else:
        i = moji.index("{") + 1
        moji = moji[:i] + f'\n  "{namae}": {atai},' + moji[i:]
    json.loads(moji)                      # 壊れた JSON にしていないか確かめてから書く
    with open(KYOTSU_FILE, "w", encoding="utf-8") as f:
        f.write(moji)


def settei_kesu(namae):
    """kuruma_settei.json から値を消す（共通設定の値が効くようにするため）"""
    if not os.path.exists(SETTEI_FILE):
        return False
    try:
        with open(SETTEI_FILE) as f:
            settei = json.load(f)
    except Exception:
        return False
    if namae not in settei:
        return False
    del settei[namae]
    with open(SETTEI_FILE, "w") as f:
        json.dump(settei, f, indent=2, ensure_ascii=False)
    return True


# ==============================================================================
# 部品1: 画素 ⇔ 床の位置 の変換
# ==============================================================================

class Henkan:
    def __init__(self, H, size):
        """H: 画素(x,y) → 床(x前,y左)[m] の変換行列（3×3）。size: (画像の横, 縦)[画素]"""
        self.H = np.array(H, dtype=np.float64).reshape(3, 3)
        self.Hinv = np.linalg.inv(self.H)
        self.size = (int(size[0]), int(size[1]))

    def px_kara_yuka(self, px):
        """画素 (N,2) → 床の位置 (N,2)[m]"""
        p = np.atleast_2d(np.asarray(px, dtype=np.float64))
        q = np.column_stack([p, np.ones(len(p))]) @ self.H.T
        return q[:, :2] / q[:, 2:3]

    def yuka_kara_px(self, yuka):
        """床の位置 (N,2)[m] → 画素 (N,2) と「カメラの前にあるか」(N,)"""
        p = np.atleast_2d(np.asarray(yuka, dtype=np.float64))
        q = np.column_stack([p, np.ones(len(p))]) @ self.Hinv.T
        mae = q[:, 2] * np.sign(self._w_muki()) > 1e-9
        w = np.where(np.abs(q[:, 2]) < 1e-12, 1e-12, q[:, 2])
        return q[:, :2] / w[:, None], mae

    def _w_muki(self):
        # 画面の下中央（必ず床が映っている所）で、向きの符号を決めておく
        if not hasattr(self, "_muki"):
            shita = np.array([self.size[0] / 2.0, self.size[1] - 1.0, 1.0])
            yuka = self.H @ shita
            yuka = yuka / yuka[2]
            self._muki = float((self.Hinv @ yuka)[2])
        return self._muki

    def mieru_kyori(self):
        """画面の下端と上端の中央が、床の上で何m先にあたるか (近い方, 遠い方)。
           上端が地平線より上なら、遠い方は None"""
        w, h = self.size
        chikai = float(self.px_kara_yuka([[w / 2.0, h - 1.0]])[0, 0])
        q = self.H @ np.array([w / 2.0, 0.0, 1.0])
        tooi = None
        if q[2] * (self.H @ np.array([w / 2.0, h - 1.0, 1.0]))[2] > 0:
            tooi = float(q[0] / q[2])
            if tooi < chikai:
                tooi = None
        return chikai, tooi

    def lista(self):
        return [round(float(v), 9) for v in self.H.ravel()]

    @staticmethod
    def shikaku_kara(px4, haba, okuyuki, kyori, yoko, size):
        """床に置いた四角形の4つの角（画面でクリックした画素）から変換を作る。
             haba    : 四角形の横幅[m]
             okuyuki : 四角形の奥行き[m]
             kyori   : 後輪の車軸から、四角形の手前の辺までの距離[m]
             yoko    : 四角形の中心が、車の中心線から左へどれだけずれているか[m]（ふつう0）
           角はどの順番でクリックしてもよい（手前2つ・奥2つを自動で見分ける）。
           返り値: (Henkan, None)。うまく作れないときは (None, 理由の文)"""
        p = np.asarray(px4, dtype=np.float64).reshape(-1, 2)
        if len(p) != 4:
            return None, "角を4つクリックしてください"
        if min(haba, okuyuki) <= 0.01 or kyori <= 0:
            return None, "横幅・奥行き・距離は0より大きい値にしてください"
        jun = p[np.argsort(-p[:, 1])]                 # 画面の下にあるものほど手前
        temae = jun[:2][np.argsort(jun[:2, 0])]       # 手前の左・右
        oku = jun[2:][np.argsort(jun[2:, 0])]         # 奥の左・右
        if abs(temae[1, 0] - temae[0, 0]) < 8 or abs(oku[1, 0] - oku[0, 0]) < 8:
            return None, "角どうしが近すぎます。四角形の4つの角をクリックし直してください"
        if min(temae[:, 1]) - max(oku[:, 1]) < 8:
            return None, "手前の辺と奥の辺が見分けられません。もっと奥行きのある四角形を使ってください"
        gaso = np.array([temae[0], temae[1], oku[1], oku[0]], dtype=np.float32)
        yuka = np.array([[kyori, yoko + haba / 2], [kyori, yoko - haba / 2],
                         [kyori + okuyuki, yoko - haba / 2],
                         [kyori + okuyuki, yoko + haba / 2]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(gaso, yuka)
        if not np.all(np.isfinite(H)) or abs(np.linalg.det(H)) < 1e-12:
            return None, "変換を計算できませんでした。角をクリックし直してください"
        return Henkan(H, size), None


def henkan_yomu(p):
    """設定から変換を読む。まだキャリブレーションしていなければ None"""
    H, size = p.get("PP_HENKAN"), p.get("PP_HENKAN_SIZE")
    try:
        if H is None or size is None or len(H) != 9:
            return None
        return Henkan(H, size)
    except Exception:
        return None


# ==============================================================================
# 部品2: 点群の検出（同心円スキャン）
#
#   後輪の車軸を中心に、半径を少しずつ大きくした円を床の上に考える。
#   それぞれの円に沿ってラインの色を探し、ひとつ内側の円で見つけた位置の
#   近くにあるものを「ラインの続き」として採用する。
#   画面の行ごとに探す今の「たどり」と違い、ラインが横向きになっても追える。
# ==============================================================================

class Tengun:
    R_KIZAMI = 0.03           # 円の半径のきざみ[m]
    R_SAISHO, R_SAIDAI = 0.12, 1.60
    KAKU_SAIDAI = 110.0       # 左右に何度まで探すか
    KAKU_KIZAMI = 0.5         # 円に沿って何度おきに調べるか
    NUKE = 3                  # ラインが途切れても、円をこの数までは飛び越える
    HABA_SEN = 0.12           # これより長く円と重なっていたら「ラインが円に沿っている」とみなす[m]
    HABA_JOGEN = 0.60         # これより長いものはラインではない（大きな黒い物など）[m]

    def __init__(self, henkan, gate=0.25):
        self.henkan = henkan
        self.gate = gate
        haba, takasa = henkan.size
        r = np.arange(self.R_SAISHO, self.R_SAIDAI + 1e-9, self.R_KIZAMI)
        kaku = np.radians(np.arange(-self.KAKU_SAIDAI, self.KAKU_SAIDAI + 1e-9, self.KAKU_KIZAMI))
        xx = r[:, None] * np.cos(kaku)[None, :]
        yy = r[:, None] * np.sin(kaku)[None, :]
        px, mae = henkan.yuka_kara_px(np.column_stack([xx.ravel(), yy.ravel()]))
        ix = np.round(px[:, 0]).astype(np.int64)
        iy = np.round(px[:, 1]).astype(np.int64)
        naka = mae & (ix >= 1) & (ix < haba - 1) & (iy >= 1) & (iy < takasa - 1)
        naka = naka.reshape(len(r), len(kaku))
        tsukau = naka.sum(axis=1) >= 6          # ほとんど映っていない円は使わない
        self.r = r[tsukau]
        self.kaku = kaku
        self.naka = naka[tsukau]
        self.ix = np.where(naka, ix.reshape(naka.shape), 0)[tsukau]
        self.iy = np.where(naka, iy.reshape(naka.shape), 0)[tsukau]
        self.d_kaku = math.radians(self.KAKU_KIZAMI)

    def sagasu(self, sen_dake, tane):
        """sen_dake: ラインの色の所だけ白(255)にした画像。tane: 探し始める床の位置 (x,y)。
           返り値: 見つけた点の列 [(x, y), ...]（車に近い順）"""
        if len(self.r) == 0:
            return []
        atari = (sen_dake[self.iy, self.ix] > 0) & self.naka
        mae_x, mae_y = float(tane[0]), float(tane[1])
        r_tane = math.hypot(mae_x, mae_y)
        ten = []
        nuke = 0
        for i in range(len(self.r)):
            r = float(self.r[i])
            if not ten and r < r_tane - 0.06:
                continue                          # 探し始めの位置より手前の円は飛ばす
            gyo = atari[i]
            hashi = np.flatnonzero(np.diff(np.concatenate(([0], gyo.view(np.int8), [0]))))
            mae_kaku = math.atan2(mae_y, mae_x)
            erabi = None
            for a, b in zip(hashi[0::2], hashi[1::2] - 1):
                k0, k1 = float(self.kaku[a]), float(self.kaku[b])
                nagasa = r * (k1 - k0 + self.d_kaku)
                if nagasa > self.HABA_JOGEN:
                    continue                      # 大きすぎる物はラインではない
                if nagasa <= self.HABA_SEN:
                    k = (k0 + k1) / 2             # ふつうに横切っている → 真ん中
                else:
                    k = min(max(mae_kaku, k0), k1)  # 円に沿っている → 前の点に近い所
                x, y = r * math.cos(k), r * math.sin(k)
                d = math.hypot(x - mae_x, y - mae_y)
                if erabi is None or d < erabi[0]:
                    erabi = (d, x, y)
            # 前の点から離れすぎているものは「続き」とみなさない
            # （直角に近い角を曲がった先は 20cm ほど離れるので、ゲートは広めにしてある）
            if erabi is not None and erabi[0] > self.gate + 0.05 * nuke:
                erabi = None
            if erabi is None:
                if ten:
                    nuke += 1
                    if nuke > self.NUKE:
                        break
                elif r > r_tane + 0.25:
                    break                         # 探し始めの近くにラインが無い
                continue
            _, mae_x, mae_y = erabi
            ten.append((mae_x, mae_y))
            nuke = 0
        return ten


# ==============================================================================
# 部品2'（新）: 足元からたどる検出（Kenshutsu）
#
#   画面の「帯」には頼らず、次の順でラインだけを取り出す。
#     1. 映像を「真上から見た図」に直す（床の上の寸法で、ものの幅を測れるようにする）
#     2. 周りの床より明るい（黒線なら暗い）所だけを残す（床の明るさが変わっても効く）
#     3. 幅が PP_HABA_MIN〜PP_HABA_MAX の細長いものだけを残す
#        （細かいゴミ・床の反射のような太い塊を消す）
#     4. つながった塊ごとに分け、車の足元（前のコマのラインの位置）に届いている塊を
#        「ライン」として1つだけ選ぶ → つながっていない明るい所は、そもそも候補にならない
#     5. 選んだ塊の中だけを、同心円で奥へたどって点の列にする（Tengun と同じ）
# ==============================================================================

class Kenshutsu:
    BAISOKU = 150.0           # 真上から見た図の細かさ[画素/m]（1画素＝約6.7mm）
    Y_HAN = 0.80              # 左右に何mまで見るか
    X_OKU = 1.60              # 前に何mまで見るか
    SHOSA = 0.5               # 先に画像を何倍に縮めるか（軽くするため）
    SEED_OKU = 0.60           # 足元とみなす範囲: 画面に映る一番手前から、前にこの距離[m]まで
    MIN_MENSEKI = 60          # 小さすぎる塊は無視[画素]（150画素/m で約 2.7cm 角）
    MIN_SEED = 12             # 足元の範囲の中に、塊がこの画素数以上入っていること

    def __init__(self, henkan, p):
        self.henkan = henkan
        self.p = p
        chikai, _ = henkan.mieru_kyori()
        self.x_lo = max(0.0, chikai - 0.02)
        b = self.BAISOKU
        self.W = int(round(2 * self.Y_HAN * b))
        self.Hh = int(round((self.X_OKU - self.x_lo) * b))
        self.w_s = max(2, int(round(henkan.size[0] * self.SHOSA)))
        self.h_s = max(2, int(round(henkan.size[1] * self.SHOSA)))
        S = np.diag([self.w_s / henkan.size[0], self.h_s / henkan.size[1], 1.0])
        A = np.array([[0.0, -b, self.Y_HAN * b],
                      [-b, 0.0, self.X_OKU * b],
                      [0.0, 0.0, 1.0]])
        self.M = A @ henkan.H @ np.linalg.inv(S)          # 縮めた画像の画素 → 真上から見た図の画素
        self.valid = cv2.erode(cv2.warpPerspective(np.full((self.h_s, self.w_s), 255, np.uint8),
                                                   self.M, (self.W, self.Hh), flags=cv2.INTER_NEAREST),
                               np.ones((5, 5), np.uint8)) > 0
        self.settei_kaeru(p)
        self.mask = None          # 掃除したあとの、ラインらしいもの全部
        self.sentaku = None       # そのうち、ラインとして選んだ塊
        self.sentaku_zumi = False

    def settei_kaeru(self, p):
        self.p = p
        b = self.BAISOKU
        ku = lambda m: max(3, int(round(m * b)) | 1)       # m → 奇数の画素数
        self.k_tophat = np.ones((ku(2.5 * float(p["PP_HABA_MAX"])),) * 2, np.uint8)
        self.k_blob = np.ones((ku(1.5 * float(p["PP_HABA_MAX"])),) * 2, np.uint8)
        self.k_blob_hiroge = np.ones((5, 5), np.uint8)

    def yuka_kara_px(self, x, y):
        """床の位置[m] → 真上から見た図の (列, 行)"""
        return ((self.Y_HAN - y) * self.BAISOKU, (self.X_OKU - x) * self.BAISOKU)

    def shori(self, gray, line_iro, shikii):
        """グレー画像 → 「ラインらしい所」だけ白(255)にした、真上から見た図"""
        h, w = gray.shape[:2]
        if (w, h) != tuple(self.henkan.size):
            gray = cv2.resize(gray, tuple(self.henkan.size), interpolation=cv2.INTER_AREA)
        small = cv2.resize(gray, (self.w_s, self.h_s), interpolation=cv2.INTER_AREA)
        if line_iro == "kuro":
            small = 255 - small                 # 黒線は、明暗を逆にして「明るい線」として扱う
            kijun = 255 - int(shikii)
        else:
            kijun = int(shikii)
        b = cv2.warpPerspective(small, self.M, (self.W, self.Hh), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        # 周りの床より明るい所だけ（細いものだけ残す top-hat 変換）
        th = cv2.morphologyEx(b, cv2.MORPH_TOPHAT, self.k_tophat)
        m = ((b >= kijun) & (th >= int(self.p["PP_SA"])) & self.valid).astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))      # 細かいゴミ
        # 太すぎる塊（床の反射など）を消す
        ookii = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k_blob)
        if ookii.any():
            ookii = cv2.dilate(ookii, self.k_blob_hiroge)
            m[ookii > 0] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))     # 小さな途切れをつなぐ
        self.mask = m
        return m

    def erabu(self, mask, y_chuo, y_han):
        """足元に届いている塊のうち、前のコマのラインの位置(y_chuo)に一番近いものを1つ選ぶ。
           返り値: (その塊だけの図, 探し始めの床の位置(x, y)) / 見つからなければ (None, None)"""
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        self.sentaku = None
        if n <= 1:
            return None, None
        b = self.BAISOKU
        r1 = self.Hh
        r0 = max(0, int(round((self.X_OKU - (self.x_lo + self.SEED_OKU)) * b)))
        c0 = max(0, int(round((self.Y_HAN - (y_chuo + y_han)) * b)))
        c1 = min(self.W, int(round((self.Y_HAN - (y_chuo - y_han)) * b)))
        if c1 - c0 < 2 or r1 - r0 < 2:
            return None, None
        win = lab[r0:r1, c0:c1]
        erabi = None
        for i in np.unique(win[win > 0]):
            if st[i, cv2.CC_STAT_AREA] < self.MIN_MENSEKI:
                continue
            gyo, retsu = np.nonzero(win == i)
            if len(gyo) < self.MIN_SEED:
                continue
            y_hei = self.Y_HAN - (retsu.mean() + c0) / b
            sa = abs(y_hei - y_chuo)
            if erabi is None or sa < erabi[0]:
                erabi = (sa, i, gyo, retsu)
        if erabi is None:
            return None, None
        _, i, gyo, retsu = erabi
        chikai = gyo >= gyo.max() - 4                    # 一番手前の数行
        col = retsu[chikai].mean() + c0
        row = gyo[chikai].mean() + r0
        tane = (self.X_OKU - row / b, self.Y_HAN - col / b)
        self.sentaku = (lab == i).astype(np.uint8) * 255
        return self.sentaku, tane

    def gazou_ni_modosu(self, size, sentaku_nomi=False):
        """掃除したあとの図を、カメラ映像の位置に戻した画像（確認用）。
           ラインとして選んだ塊=255、ほかのラインらしい所=90、それ以外=0"""
        if self.mask is None:
            return np.zeros((size[1], size[0]), np.uint8)
        e = np.zeros((self.h_s, self.w_s), np.uint8)
        flags = cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
        if not sentaku_nomi:
            e = cv2.warpPerspective(self.mask, self.M, (self.w_s, self.h_s), flags=flags)
            e = (e > 0).astype(np.uint8) * 90
        if self.sentaku is not None:
            s = cv2.warpPerspective(self.sentaku, self.M, (self.w_s, self.h_s), flags=flags)
            e[s > 0] = 255
        return cv2.resize(e, tuple(size), interpolation=cv2.INTER_NEAREST)


class TengunTori(Tengun):
    """Tengun と同じ「同心円でたどる」やり方で、真上から見た図の上を探す"""

    def __init__(self, ken, gate=0.25):
        self.gate = gate
        r = np.arange(self.R_SAISHO, self.R_SAIDAI + 1e-9, self.R_KIZAMI)
        kaku = np.radians(np.arange(-self.KAKU_SAIDAI, self.KAKU_SAIDAI + 1e-9, self.KAKU_KIZAMI))
        xx = r[:, None] * np.cos(kaku)[None, :]
        yy = r[:, None] * np.sin(kaku)[None, :]
        col, row = ken.yuka_kara_px(xx, yy)
        ix = np.round(col).astype(np.int64)
        iy = np.round(row).astype(np.int64)
        naka = (xx >= ken.x_lo) & (ix >= 0) & (ix < ken.W) & (iy >= 0) & (iy < ken.Hh)
        tsukau = naka.sum(axis=1) >= 6
        self.r = r[tsukau]
        self.kaku = kaku
        self.naka = naka[tsukau]
        self.ix = np.where(naka, ix, 0)[tsukau]
        self.iy = np.where(naka, iy, 0)[tsukau]
        self.d_kaku = math.radians(self.KAKU_KIZAMI)


# ==============================================================================
# 部品3: ピュアパシュート
# ==============================================================================

class PurePursuit:
    def __init__(self, p):
        """p: 設定の辞書（settei_toridasu で作る）"""
        self.mae_tane = None      # 前のコマで見つけた一番手前の点（探し始めの位置の予備）
        self.mae_tane_kazu = 99
        self.y_ato = None         # 新しい検出: 前のコマで見つけたラインの足元の位置（左+）[m]
        self.miushinai = 99       # 新しい検出: ラインが見つからなかったコマの連続数
        self.settei_kaeru(p)

    def settei_kaeru(self, p):
        self.p = dict(SHOKICHI)
        self.p.update({k: v for k, v in p.items() if k in SHOKICHI})
        self.henkan = henkan_yomu(self.p)
        self.tengun = None
        self.ken = None
        self.tengun_tori = None
        if self.henkan is not None:
            self.tengun = Tengun(self.henkan, float(self.p["PP_GATE"]))
            self.ken = Kenshutsu(self.henkan, self.p)
            self.tengun_tori = TengunTori(self.ken, float(self.p["PP_GATE"]))

    @property
    def tsukaeru(self):
        """キャリブレーションが済んでいて使えるか"""
        return self.tengun is not None

    def sakiyomi_kyori(self, accel):
        """先読み距離 Ld[m]。速く走るほど遠くを見る。
           速度はセンサーが無いので「アクセル% × PP_SOKUDO_KEISU」で見積もる"""
        sokudo = max(0.0, float(self.p["PP_SOKUDO_KEISU"]) * float(accel or 0.0))
        ld = float(self.p["PP_LD_KIHON"]) + float(self.p["PP_LD_SOKUDO"]) * sokudo
        return max(float(self.p["PP_LD_SAISHO"]), ld), sokudo

    def keisan(self, sen_dake, zure_px=None, y_px=None, accel=0.0):
        """1コマぶんの計算。
             sen_dake : ラインの色の所だけ白(255)にした画像
             zure_px  : 今の方式が見つけた近くの帯のズレ[px]（探し始めの位置に使う。無ければ None）
             y_px     : その帯の高さ[px]
             accel    : いまのアクセル[%]（先読み距離を決めるのに使う）
           返り値の辞書:
             ok       : 目標点が決まったか（False のときは今の方式で走ること）
             sa       : ハンドルの量（-1〜+1。+が左。1 で左右いっぱい）
             kakudo   : タイヤの切れ角[°]（+が左）
             gen      : カーブでアクセルを緩める割合（1=そのまま、0.5=半分）
             ten, mokuhyo, ld, todoita, kyokuritsu : 画面表示・記録用"""
        kekka = {"ok": False, "sa": 0.0, "kakudo": 0.0, "gen": 1.0, "ten": [], "mokuhyo": None,
                 "ld": None, "todoita": None, "kyokuritsu": None, "sokudo": 0.0}
        if self.tengun is None:
            return kekka
        ld, sokudo = self.sakiyomi_kyori(accel)
        kekka["ld"], kekka["sokudo"] = round(ld, 3), round(sokudo, 2)

        # --- 探し始めの位置: 今の方式が追っているライン → 無ければ前のコマの位置 ---
        tane = None
        if zure_px is not None and y_px is not None:
            t = self.henkan.px_kara_yuka([[self.henkan.size[0] / 2.0 + zure_px, y_px]])[0]
            if 0 < t[0] < 3.0:
                tane = (float(t[0]), float(t[1]))
        yobi = False              # 前のコマの位置（予備）から探したか
        if tane is None and self.mae_tane is not None and self.mae_tane_kazu <= 10:
            tane = self.mae_tane
            yobi = True
        self.mae_tane_kazu += 1
        if tane is None:
            return kekka

        ten = self.tengun.sagasu(sen_dake, tane)
        kekka["ten"] = [(round(x, 3), round(y, 3)) for x, y in ten]
        if len(ten) < 2:
            return kekka
        # 予備の位置から見つけた点は、次のコマの予備にしない。
        # 予備を予備でつなぐと、ラインを見失った後も床の白い点々などを
        # どこまでも追いかけてしまう（予備は最後にラインが見えたコマから 10 コマまで）
        if not yobi:
            self.mae_tane, self.mae_tane_kazu = ten[0], 0
        t = np.array(ten)
        r = np.hypot(t[:, 0], t[:, 1])
        kekka["todoita"] = round(float(r[-1]), 3)
        if r[-1] < float(self.p["PP_LD_SAISHO"]) * 0.6:
            return kekka                      # 近くしか見えていない → 今の方式に任せる

        # --- 目標点: 半径 Ld の円とラインが交わる所（届かなければ一番遠い点） ---
        k = int(np.argmin(np.abs(r - ld))) if r[-1] >= ld else len(r) - 1
        k0, k1 = max(0, k - 1), min(len(r), k + 2)
        mx, my = float(t[k0:k1, 0].mean()), float(t[k0:k1, 1].mean())   # 前後の点と平均してブレを減らす
        kekka["mokuhyo"] = (round(mx, 3), round(my, 3))

        # --- ピュアパシュートの式: 目標点を通る円の曲がり具合 → タイヤの切れ角 ---
        kyoku = 2.0 * my / (mx * mx + my * my)
        kaku = math.degrees(math.atan(float(self.p["WHEELBASE"]) * kyoku))
        saidai = max(1.0, float(self.p["KIREKAKU_DO"]))
        kekka["kakudo"] = round(kaku, 2)
        kekka["sa"] = max(-1.0, min(1.0, kaku / saidai))
        kekka["kyokuritsu"] = round(kyoku, 3)

        # --- カーブの手前でアクセルを緩める（遠くの点へ向かう円の曲がり具合で決める） ---
        gen_gain = float(self.p["PP_CURVE_GEN"])
        if gen_gain > 0:
            saidai_k = abs(kyoku)
            for rr in (0.6, 0.85, 1.1):
                j = int(np.argmax(r >= rr)) if r[-1] >= rr else len(r) - 1
                saidai_k = max(saidai_k, abs(2.0 * t[j, 1] / (r[j] ** 2)))
            kekka["gen"] = round(max(float(self.p["PP_CURVE_SAITEI"]), min(1.0, 1.0 - gen_gain * saidai_k)), 3)
        kekka["ok"] = True
        return kekka

    # --------------------------------------------------------------------------
    # 新しい検出（足元からたどる）で 1コマぶんを計算する。rc_race.py と pp_test.py はこちらを使う
    # --------------------------------------------------------------------------
    def _mokuhyo(self, t, r, ld):
        """点の列 t（床の位置）と、車軸からの距離 r から、半径 ld の円と交わる所を目標点にする"""
        k = int(np.argmin(np.abs(r - ld))) if r[-1] >= ld else len(r) - 1
        k0, k1 = max(0, k - 1), min(len(r), k + 2)
        return float(t[k0:k1, 0].mean()), float(t[k0:k1, 1].mean())

    def keisan_gazou(self, gray, line_iro, shikii, accel=0.0):
        """gray: グレー画像。line_iro: "shiro"/"kuro"。shikii: その色用のしきい値。
           返り値の辞書は keisan() と同じ（ok / sa / kakudo / gen / ten / mokuhyo / ld / todoita /
           kyokuritsu）に、次を足したもの:
             joutai : "ok" / "line_nashi"（足元にラインが無い）/ "ten_sukunai" / "keisan_fuka"
             tane   : 探し始めの床の位置 (x, y)
             ld0    : カーブで縮める前の先読み距離"""
        kekka = {"ok": False, "sa": 0.0, "kakudo": 0.0, "gen": 1.0, "ten": [], "mokuhyo": None,
                 "ld": None, "todoita": None, "kyokuritsu": None, "sokudo": 0.0,
                 "joutai": "keisan_fuka", "tane": None, "ld0": None}
        if self.ken is None or self.tengun_tori is None:
            return kekka
        ld0, sokudo = self.sakiyomi_kyori(accel)
        kekka["ld0"], kekka["sokudo"] = round(ld0, 3), round(sokudo, 2)
        kekka["ld"] = kekka["ld0"]

        # --- 1〜3: 真上から見た図にして、ラインらしい所だけ残す ---
        mask = self.ken.shori(gray, line_iro, shikii)

        # --- 4: 足元に届いている塊を、ラインとして1つ選ぶ ---
        # 前のコマで見つけたラインの位置の近くだけ探す（急に別の所へ飛ばない）。
        # 長く見失ったら、車の正面から探し直す（置き直しに追従）
        if self.miushinai > 15:
            self.y_ato = None
        y_chuo = 0.0 if self.y_ato is None else self.y_ato
        y_han = float(self.p["PP_GATE_Y"])
        sentaku, tane = self.ken.erabu(mask, y_chuo, y_han)
        if tane is None:
            self.miushinai += 1
            kekka["joutai"] = "line_nashi"
            return kekka
        self.miushinai = 0
        self.y_ato = tane[1]
        kekka["tane"] = (round(tane[0], 3), round(tane[1], 3))

        # --- 5: 選んだ塊の中だけをたどる ---
        ten = self.tengun_tori.sagasu(sentaku, tane)
        kekka["ten"] = [(round(x, 3), round(y, 3)) for x, y in ten]
        if len(ten) < 2:
            kekka["joutai"] = "ten_sukunai"
            return kekka
        t = np.array(ten)
        r = np.hypot(t[:, 0], t[:, 1])
        kekka["todoita"] = round(float(r[-1]), 3)
        if r[-1] - r[0] < 0.08:
            kekka["joutai"] = "ten_sukunai"       # 足元に少し見えているだけ → 向きが決められない
            return kekka

        # --- 先読み距離: カーブがきついほど縮める ---
        mx, my = self._mokuhyo(t, r, ld0)
        kyoku0 = abs(2.0 * my / (mx * mx + my * my))
        ld = ld0 / (1.0 + float(self.p["PP_LD_CURVE"]) * kyoku0)
        ld = max(float(self.p["PP_LD_SAISHO"]), ld, float(r[0]))     # 画面に映る一番手前より近くは見えない
        kekka["ld"] = round(ld, 3)
        mx, my = self._mokuhyo(t, r, ld)
        kekka["mokuhyo"] = (round(mx, 3), round(my, 3))

        # --- ピュアパシュートの式 ---
        kyoku = 2.0 * my / (mx * mx + my * my)
        kaku = math.degrees(math.atan(float(self.p["WHEELBASE"]) * kyoku))
        saidai = max(1.0, float(self.p["KIREKAKU_DO"]))
        kekka["kakudo"] = round(kaku, 2)
        kekka["sa"] = max(-1.0, min(1.0, kaku / saidai))
        kekka["kyokuritsu"] = round(kyoku, 3)

        gen_gain = float(self.p["PP_CURVE_GEN"])
        if gen_gain > 0:
            saidai_k = abs(kyoku)
            for rr in (0.6, 0.85, 1.1):
                j = int(np.argmax(r >= rr)) if r[-1] >= rr else len(r) - 1
                saidai_k = max(saidai_k, abs(2.0 * t[j, 1] / (r[j] ** 2)))
            kekka["gen"] = round(max(float(self.p["PP_CURVE_SAITEI"]), min(1.0, 1.0 - gen_gain * saidai_k)), 3)
        kekka["ok"] = True
        kekka["joutai"] = "ok"
        return kekka


# ==============================================================================
# 画面に描く
# ==============================================================================

def sen_dake_tsukuru(gazou, line_iro, kuro_shikii, shiro_shikii):
    """ラインの色の所だけ白(255)にした画像を作る（rc_race.py の me_de_miru と同じ決め方）"""
    shirokuro = cv2.cvtColor(gazou, cv2.COLOR_BGR2GRAY)
    if line_iro == "kuro":
        _, sen_dake = cv2.threshold(shirokuro, kuro_shikii, 255, cv2.THRESH_BINARY_INV)
    else:
        _, sen_dake = cv2.threshold(shirokuro, shiro_shikii, 255, cv2.THRESH_BINARY)
    return sen_dake


def gamen_ni_kaku(img, henkan, pp, tsukatteru=True):
    """カメラ映像に、点群（水色の点）・先読みの円（水色の線）・目標点（赤紫の丸）を描き込む"""
    if henkan is None or not pp:
        return img
    iro = (255, 230, 0) if tsukatteru else (160, 150, 90)
    if pp.get("ten"):
        px, mae = henkan.yuka_kara_px(pp["ten"])
        for (x, y), m in zip(px, mae):
            if m and -50 < x < 5000 and -50 < y < 5000:
                cv2.circle(img, (int(x), int(y)), 3, iro, -1)
    if pp.get("ld"):
        kaku = np.radians(np.arange(-80, 81, 4))
        en = np.column_stack([pp["ld"] * np.cos(kaku), pp["ld"] * np.sin(kaku)])
        px, mae = henkan.yuka_kara_px(en)
        px = px[mae & (np.abs(px[:, 0]) < 5000) & (np.abs(px[:, 1]) < 5000)]
        if len(px) >= 2:
            cv2.polylines(img, [px.astype(np.int32).reshape(-1, 1, 2)], False, iro, 1, cv2.LINE_AA)
    if pp.get("mokuhyo"):
        px, mae = henkan.yuka_kara_px([pp["mokuhyo"]])
        if mae[0]:
            c = (int(px[0, 0]), int(px[0, 1]))
            cv2.circle(img, c, 9, (255, 0, 255) if tsukatteru else (150, 90, 150), 2)
            cv2.circle(img, c, 2, (255, 0, 255) if tsukatteru else (150, 90, 150), -1)
    return img


def tori_zu(gazou, henkan, pp=None, shikaku=None, baisoku=260, x_han=(-0.15, 1.60), y_han=0.80):
    """「真上から見た図」を作る。カメラ映像を床に貼りつけ直した絵に、
       10cm のマス目・車の位置・点群・目標点・進む円弧を描く。
       キャリブレーションが正しければ、床に置いた四角形がちゃんと四角に見える。
         shikaku: キャリブレーションに使った四角形の床での角 [(x,y)×4]（あれば枠を描く）"""
    haba = int(round(2 * y_han * baisoku))
    takasa = int(round((x_han[1] - x_han[0]) * baisoku))

    def e(x, y):                         # 床の位置 → この図の画素
        return (int(round((y_han - y) * baisoku)), int(round((x_han[1] - x) * baisoku)))

    A = np.array([[0.0, -baisoku, y_han * baisoku],
                  [-baisoku, 0.0, x_han[1] * baisoku],
                  [0.0, 0.0, 1.0]])
    zu = cv2.warpPerspective(gazou, A @ henkan.H, (haba, takasa),
                             flags=cv2.INTER_LINEAR, borderValue=(40, 40, 40))
    # マス目（10cm ごと。50cm ごとは濃く、数字つき）
    for i in range(int(round(x_han[0] * 10)), int(round(x_han[1] * 10)) + 1):
        x = i / 10.0
        koi = i % 5 == 0
        cv2.line(zu, e(x, y_han), e(x, -y_han), (90, 200, 90) if koi else (70, 110, 70), 1)
        if koi and i > 0:
            cv2.putText(zu, f"{x:.1f}m", (4, e(x, 0)[1] - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (120, 255, 120), 1, cv2.LINE_AA)
    for i in range(int(round(-y_han * 10)), int(round(y_han * 10)) + 1):
        y = i / 10.0
        cv2.line(zu, e(x_han[0], y), e(x_han[1], y),
                 (90, 200, 90) if i % 5 == 0 else (70, 110, 70), 2 if i == 0 else 1)
    # 車（後輪の車軸が原点。前輪までを四角で描く）
    cv2.rectangle(zu, e(0.30, 0.09), e(-0.05, -0.09), (255, 180, 60), 2)
    cv2.circle(zu, e(0, 0), 4, (255, 180, 60), -1)
    if shikaku is not None:
        cv2.polylines(zu, [np.array([e(x, y) for x, y in shikaku], np.int32).reshape(-1, 1, 2)],
                      True, (0, 200, 255), 2, cv2.LINE_AA)
    if pp:
        for x, y in pp.get("ten") or []:
            cv2.circle(zu, e(x, y), 3, (255, 230, 0), -1)
        if pp.get("ld"):
            kaku = np.radians(np.arange(-100, 101, 4))
            en = [e(pp["ld"] * math.cos(k), pp["ld"] * math.sin(k)) for k in kaku]
            cv2.polylines(zu, [np.array(en, np.int32).reshape(-1, 1, 2)], False, (255, 230, 0), 1, cv2.LINE_AA)
        if pp.get("mokuhyo") and pp.get("ok"):
            mx, my = pp["mokuhyo"]
            k = pp.get("kyokuritsu") or 0.0
            # 進む円弧（原点から目標点まで）
            if abs(k) > 1e-3:
                hankei = 1.0 / k
                owari = 2 * math.asin(max(-1.0, min(1.0, math.hypot(mx, my) * abs(k) / 2)))
                ko = [e(abs(hankei) * math.sin(a), hankei * (1 - math.cos(a)))
                      for a in np.linspace(0, owari, 24)]
            else:
                ko = [e(0, 0), e(mx, my)]
            cv2.polylines(zu, [np.array(ko, np.int32).reshape(-1, 1, 2)], False, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(zu, e(mx, my), 9, (255, 0, 255), 2)
    return zu
