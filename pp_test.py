#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   pp_test.py — ピュアパシュートの準備と確認
#
#   ハンドルの新方式「ピュアパシュート」を使う前の準備をするプログラムです。
#   モーターもハンドルも動かしません（計算と表示だけ）。安心して試せます。
#   詳しい説明は docs/説明書_pp_test.md を読んでください。
#
# ==============================================================================
#
#  ● このプログラムでやること
#
#      1. キャリブレーション … 床に置いた四角形を使って、カメラの画面の位置と
#                             床の上の位置[m]の対応を覚えさせる（最初に1回）
#      2. 見え方の確認     … 「真上から見た図」で、ラインの点群と目標点を見る
#      3. 値の調整         … 先読み距離などを変えて、今の方式との違いを見比べる
#      4. 方式の切り替え    … 本番（rc_race.py）で使う方式を選ぶ
#
#  ● 使い方
#
#      1. ラズパイのターミナルで:   python3 pp_test.py
#      2. ブラウザで:  http://<この車のIPアドレス>:8080
#      3. 終わるときはターミナルで Ctrl+C
#
#      rc_race.py などと同じ 8080 番を使うので、同時には起動できません。
#
# ==============================================================================

import json
import time
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import cv2

import rc_race            # カメラ・ライン検出・設定は本番と同じものを使う（読み込むだけでは走らない）
import line_algo

HTTP_PORT = rc_race.HTTP_PORT
HAISHIN_FPS = rc_race.HAISHIN_FPS
HAISHIN_GASHITSU = rc_race.HAISHIN_GASHITSU

# ラインと判定するしきい値（ラインの色で使う値が変わる）
if rc_race.LINE_IRO == "kuro":
    SHIKII = ("KURO_SHIKII", "黒線と判定する暗さ（これより暗い所がライン）", 20, 160, 1)
else:
    SHIKII = ("SHIRO_SHIKII", "白線と判定する明るさ（これより明るい所がライン）", 100, 250, 1)

# 画面で変えられる値: (名前, 説明, 最小, 最大, きざみ)
CHOSEI = [
    SHIKII,
    ("PP_LD_KIHON", "先読み距離の基本 [m]", 0.20, 1.20, 0.05),
    ("PP_LD_SOKUDO", "速度1m/sあたりの伸び [m]", 0.00, 0.50, 0.05),
    ("PP_SOKUDO_KEISU", "アクセル1%あたりの速度 [m/s]", 0.00, 0.30, 0.01),
    ("KIREKAKU_DO", "ハンドル1.00の切れ角 [°]", 10.0, 40.0, 0.5),
    ("PP_CURVE_GEN", "カーブでアクセルを緩める強さ", 0.00, 0.50, 0.05),
]


class PpTestApp:
    def __init__(self):
        self.kamera = rc_race.Kamera()
        self.lock = threading.Lock()
        self.ugoiteru = True
        self.settei = line_algo.settei_toridasu(vars(rc_race))
        self.settei[SHIKII[0]] = getattr(rc_race, SHIKII[0])
        self.shirokuro = False        # 映像を「ラインと判定した所だけ白」の白黒で見せる
        self.pp = line_algo.PurePursuit(self.settei)
        self.algo = rc_race.LINE_ALGO if rc_race.LINE_ALGO in ("pd", "pp") else "pd"
        self.mihozon = False          # 画面で変えた値をまだ保存していない
        self.sotei_accel = 20.0       # 「アクセルが何%のとき」を想定して先読み距離を見るか
        # キャリブレーション
        self.calib_chu = False
        self.tometa_gazou = None      # キャリブレーション中に止めておく映像
        self.kado = []                # クリックした角 [(x, y)]（カメラ画像の画素）
        self.shikaku = getattr(rc_race, "PP_SHIKAKU", None)   # [横幅, 奥行き, 距離, 横ずれ][m]
        self.message = ""
        # ライン追跡（本番と同じ）
        self.line_ato = None
        self.zure_zenkai = None
        self.sa_pd = 0.0
        self.kekka, self.kekka_pp = {}, {}
        self.fps = 0.0
        self.cam_jpeg = None
        self.tori_jpeg = None

    # ---- キャリブレーション ----
    def calib_hajime(self):
        with self.lock:
            self.calib_chu = True
            self.kado = []
            self.tometa_gazou = None      # 次のコマを止め絵にする
            self.message = "映像を止めました。四角形の4つの角を順にクリックしてください"

    def calib_yame(self):
        with self.lock:
            self.calib_chu = False
            self.kado = []
            self.tometa_gazou = None
            self.message = ""

    def click(self, x_wari, y_wari):
        """映像の上でクリックされた位置（0〜1 の割合）を角として覚える"""
        with self.lock:
            if not self.calib_chu:
                return
            if len(self.kado) >= 4:
                self.kado = []            # 5回目のクリックで最初からやり直し
            self.kado.append((x_wari * rc_race.CAM_WIDTH, y_wari * rc_race.CAM_HEIGHT))
            nokori = 4 - len(self.kado)
            self.message = (f"角 {len(self.kado)} 個目。あと {nokori} 個" if nokori
                            else "4つそろいました。寸法を確かめて「計算して保存」を押してください")

    def calib_hozon(self, haba, okuyuki, kyori, yoko):
        with self.lock:
            if not self.calib_chu or len(self.kado) != 4:
                self.message = "先に四角形の角を4つクリックしてください"
                return
            size = (rc_race.CAM_WIDTH, rc_race.CAM_HEIGHT)
            hk, riyu = line_algo.Henkan.shikaku_kara(self.kado, haba, okuyuki, kyori, yoko, size)
            if hk is None:
                self.message = "失敗: " + riyu
                return
            chikai, tooi = hk.mieru_kyori()
            if not (0.0 < chikai < 1.5):
                self.message = (f"失敗: 画面の一番下が車軸から {chikai:.2f}m 先という計算になりました。"
                                "寸法（特に車軸からの距離）と、クリックした角を確かめてください")
                return
            self.shikaku = [haba, okuyuki, kyori, yoko]
            self.settei["PP_HENKAN"] = hk.lista()
            self.settei["PP_HENKAN_SIZE"] = list(size)
            self.pp.settei_kaeru(self.settei)
            line_algo.settei_kaku({"PP_HENKAN": self.settei["PP_HENKAN"],
                                   "PP_HENKAN_SIZE": self.settei["PP_HENKAN_SIZE"],
                                   "PP_SHIKAKU": self.shikaku})
            self.calib_chu = False
            self.kado = []
            self.tometa_gazou = None
            self.message = ("キャリブレーションを保存しました。真上から見た図で、"
                            "四角形（オレンジの枠）が床の四角形と重なっているか確かめてください")
            print(f"[保存] キャリブレーションを {line_algo.SETTEI_FILE} に保存しました"
                  f"（見える範囲: 車軸から {chikai:.2f}m 〜 {'地平線' if tooi is None else f'{tooi:.2f}m'}）")

    # ---- 値の調整 ----
    def atai_kaeru(self, namae, atai):
        with self.lock:
            if namae == "SOTEI_ACCEL":
                self.sotei_accel = max(0.0, min(100.0, float(atai)))
                return
            for n, _, saisho, saidai, _ in CHOSEI:
                if n == namae:
                    self.settei[n] = max(saisho, min(saidai, float(atai)))
                    if n == SHIKII[0]:
                        # しきい値は rc_race の検出（今の方式）と点群の両方に効かせる
                        self.settei[n] = int(round(self.settei[n]))
                        setattr(rc_race, n, self.settei[n])
                    else:
                        self.pp.settei_kaeru(self.settei)
                    self.mihozon = True
                    return

    def atai_hozon(self):
        with self.lock:
            line_algo.settei_kaku({n: self.settei[n] for n, *_ in CHOSEI})
            self.mihozon = False
            self.message = "値を保存しました（rc_race.py にも反映されます）"
            print(f"[保存] 調整した値を {line_algo.SETTEI_FILE} に保存しました")

    def shirokuro_kirikae(self):
        with self.lock:
            self.shirokuro = not self.shirokuro

    def algo_erabu(self, algo):
        with self.lock:
            if algo not in ("pd", "pp"):
                return
            if algo == "pp" and not self.pp.tsukaeru:
                self.message = "ピュアパシュートは、キャリブレーションが済むまで選べません"
                return
            self.algo = algo
            line_algo.settei_kaku({"LINE_ALGO": algo})
            self.message = ("本番の方式を「ピュアパシュート」にしました" if algo == "pp"
                            else "本番の方式を「今の方式」にしました")
            print(f"[保存] LINE_ALGO = {algo}")

    # ---- メインループ: 撮る → 見る → 計算する → 絵を作る ----
    def mawasu(self):
        mae_jikoku = time.time()
        while self.ugoiteru:
            gazou = self.kamera.toru()
            with self.lock:
                if self.calib_chu:
                    if self.tometa_gazou is None:
                        self.tometa_gazou = gazou.copy()
                    gazou = self.tometa_gazou.copy()
                ato = self.line_ato
                kado = list(self.kado)
                sotei = self.sotei_accel
                shikaku = self.shikaku

            kekka = rc_race.me_de_miru(gazou, ato)

            # 今の方式のハンドル（rc_race.py と同じ式。見比べ用）
            sa_pd = self.sa_pd
            muki = -1 if rc_race.HANDORU_GYAKU else 1
            if kekka["zure"] is not None and kekka["hirogari"] < rc_race.STEER_HOJI_HIROGARI:
                sabun = 0.0 if self.zure_zenkai is None else kekka["zure"] - self.zure_zenkai
                self.zure_zenkai = kekka["zure"]
                sa = -(rc_race.STEER_P_GAIN * kekka["nerai"] + rc_race.STEER_D_GAIN * sabun) * muki
                sa_pd = max(-rc_race.KIREKAKU_HABA, min(rc_race.KIREKAKU_HABA, sa))
            else:
                self.zure_zenkai = None

            # ピュアパシュート
            kekka_pp = {}
            sen_dake = line_algo.sen_dake_tsukuru(gazou, rc_race.LINE_IRO,
                                                  rc_race.KURO_SHIKII, rc_race.SHIRO_SHIKII)
            with self.lock:
                pp, henkan = self.pp, self.pp.henkan
                shirokuro = self.shirokuro
                if pp.tsukaeru:
                    y_c = gazou.shape[0] * (rc_race.ROI_CHIKAKU_UE + rc_race.ROI_CHIKAKU_SHITA) / 2
                    kekka_pp = pp.keisan(sen_dake, kekka["zure"], y_c, sotei)
                    kekka_pp["handoru"] = round(kekka_pp["sa"] * rc_race.KIREKAKU_HABA * muki, 2)

            # --- カメラ映像 ---
            if shirokuro:
                # ラインと判定した所を白、それ以外を黒にした絵の上に、同じ印を描く
                moto = cv2.cvtColor(sen_dake, cv2.COLOR_GRAY2BGR)
            else:
                moto = gazou
            e = rc_race.gamen_ni_kaku(moto, kekka, "PP TEST", ato)
            if henkan is not None:
                line_algo.gamen_ni_kaku(e, henkan, kekka_pp, True)
            for n, (x, y) in enumerate(kado):
                c = (int(x), int(y))
                cv2.circle(e, c, 7, (0, 165, 255), 2)
                cv2.line(e, (c[0] - 12, c[1]), (c[0] + 12, c[1]), (0, 165, 255), 1)
                cv2.line(e, (c[0], c[1] - 12), (c[0], c[1] + 12), (0, 165, 255), 1)
                cv2.putText(e, str(n + 1), (c[0] + 10, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 165, 255), 2, cv2.LINE_AA)
            if len(kado) >= 2:
                cv2.polylines(e, [np.array(kado, np.int32).reshape(-1, 1, 2)], len(kado) == 4,
                              (0, 165, 255), 1, cv2.LINE_AA)
            ok, jpeg = cv2.imencode(".jpg", e, [cv2.IMWRITE_JPEG_QUALITY, HAISHIN_GASHITSU])

            # --- 真上から見た図 ---
            tori = None
            if henkan is not None:
                kado_yuka = None
                if shikaku:
                    w, d, k, y0 = shikaku
                    kado_yuka = [(k, y0 + w / 2), (k, y0 - w / 2), (k + d, y0 - w / 2), (k + d, y0 + w / 2)]
                zu = line_algo.tori_zu(gazou, henkan, kekka_pp, kado_yuka)
                ok2, tj = cv2.imencode(".jpg", zu, [cv2.IMWRITE_JPEG_QUALITY, HAISHIN_GASHITSU])
                tori = tj.tobytes() if ok2 else None

            ima = time.time()
            dt = ima - mae_jikoku
            mae_jikoku = ima
            with self.lock:
                if kekka["line_mieru"]:
                    self.line_ato = kekka["zure"]
                elif not self.calib_chu:
                    self.line_ato = None          # 見失ったら中央から探し直す（置き直しに追従）
                self.sa_pd = sa_pd
                self.kekka, self.kekka_pp = kekka, kekka_pp
                if dt > 0:
                    self.fps = 0.9 * self.fps + 0.1 / dt if self.fps else 1.0 / dt
                if ok:
                    self.cam_jpeg = jpeg.tobytes()
                self.tori_jpeg = tori
        self.kamera.close()

    # ---- ブラウザに送る状態 ----
    def joutai_json(self):
        with self.lock:
            pp = self.kekka_pp or {}
            hk = self.pp.henkan
            chikai = tooi = None
            if hk is not None:
                chikai, tooi = hk.mieru_kyori()
            ld, sokudo = self.pp.sakiyomi_kyori(self.sotei_accel)
            return json.dumps({
                "calib_zumi": self.pp.tsukaeru,
                "calib_chu": self.calib_chu,
                "kado": len(self.kado),
                "shikaku": self.shikaku,
                "mieru_chikai": None if chikai is None else round(chikai, 2),
                "mieru_tooi": None if tooi is None else round(tooi, 2),
                "line_mieru": bool(self.kekka.get("line_mieru")),
                "pp_ok": bool(pp.get("ok")),
                "kazu": len(pp.get("ten") or []),
                "todoita": pp.get("todoita"),
                "mokuhyo": pp.get("mokuhyo"),
                "kakudo": pp.get("kakudo"),
                "gen": pp.get("gen"),
                "handoru_pp": pp.get("handoru"),
                "handoru_pd": round(self.sa_pd, 2),
                "ld": round(ld, 2),
                "sokudo": round(sokudo, 2),
                "sotei_accel": self.sotei_accel,
                "atai": {n: self.settei[n] for n, *_ in CHOSEI},
                "mihozon": self.mihozon,
                "algo": self.algo,
                "shirokuro": self.shirokuro,
                "line_iro": rc_race.LINE_IRO,
                "fps": round(self.fps, 1),
                "message": self.message,
            })


# ==============================================================================
# ブラウザの画面
# ==============================================================================

GAMEN_HTML = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PP TEST</title>
<style>
  :root { --bg:#171a1e; --panel:#21262c; --line:#343b44;
    --text:#e8eaed; --dim:#8a939e; --acc:#5b8dbf; --ok:#43b06a; --warn:#d9a441; --pp:#2f8f8f; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text);
    font-family:"Hiragino Sans","Noto Sans JP",sans-serif;
    min-height:100vh; display:flex; flex-direction:column;
    align-items:center; gap:12px; padding:14px; }
  h1 { font-size:22px; letter-spacing:1px; }
  #mode { font-size:14px; color:var(--dim); }
  #mode.ok { color:var(--ok); }
  #wrap { display:grid; grid-template-columns:1.15fr 1fr; gap:12px;
    width:100%; max-width:1240px; align-items:start; }
  .col { display:flex; flex-direction:column; gap:12px; min-width:0; }
  @media (max-width:900px) { #wrap { grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line);
    border-radius:12px; padding:16px; width:100%; }
  .card h2 { font-size:15px; color:var(--dim); margin-bottom:10px; }
  img.eizo { width:100%; display:block; border-radius:8px; background:#000; }
  #cam.click { cursor:crosshair; outline:3px solid var(--warn); }
  #tori-nashi { color:var(--dim); font-size:14px; padding:30px 10px; text-align:center; line-height:1.8; }
  .hanrei { display:flex; flex-wrap:wrap; gap:6px 14px; font-size:12px; color:var(--dim); margin-top:8px; }
  .hanrei i { display:inline-block; width:11px; height:11px; border-radius:3px; margin-right:5px; vertical-align:-1px; }
  .vals { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }
  .v { font-size:26px; font-weight:700; text-align:center; font-variant-numeric:tabular-nums; }
  .k { font-size:12px; color:var(--dim); text-align:center; margin-top:2px; }
  .hikaku { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:12px; }
  .hikaku > div { border:1px solid var(--line); border-radius:10px; padding:10px; }
  .hikaku .pp { border-color:var(--pp); }
  button { border:none; border-radius:10px; color:#fff; font-weight:700; cursor:pointer;
    font-family:inherit; padding:14px 8px; font-size:16px; }
  button:active { filter:brightness(.85); }
  button:disabled { opacity:.4; cursor:default; }
  .row { display:flex; gap:8px; margin-top:10px; }
  .row button { flex:1; }
  .b-ok { background:var(--ok); } .b-acc { background:#3b526b; } .b-gray { background:#4a4f57; }
  .b-sel { background:var(--pp); outline:3px solid #7ce0e0; }
  .sunpo { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:10px; }
  .sunpo label { font-size:12px; color:var(--dim); display:flex; flex-direction:column; gap:4px; }
  .sunpo input { padding:10px; font-size:18px; border-radius:10px; border:1px solid var(--line);
    background:#14171b; color:var(--text); text-align:center; font-variant-numeric:tabular-nums; }
  ol.tejun { font-size:13px; color:#c9cdd2; line-height:1.7; padding-left:20px; }
  .chosei { display:grid; grid-template-columns:1fr auto; gap:4px 10px; align-items:center; margin-top:8px; }
  .chosei .nm { font-size:13px; color:#c9cdd2; }
  .chosei .at { font-size:18px; font-weight:700; font-variant-numeric:tabular-nums; min-width:64px; text-align:right; }
  .chosei input[type=range] { grid-column:1 / 3; width:100%; }
  .hint { font-size:12px; color:var(--dim); margin-top:8px; line-height:1.6; }
  .chui { font-size:12px; color:#e9d9a8; background:#3a3424; border:1px solid #5a4d2a;
    border-radius:8px; padding:8px 10px; margin-top:10px; line-height:1.6; display:none; }
  #msg { text-align:center; color:var(--warn); font-size:14px; min-height:20px; }
</style>
</head>
<body>
  <h1>ピュアパシュート テスト</h1>
  <div id="mode">--</div>
  <div id="msg"></div>

  <div id="wrap">
    <div class="col">
      <div class="card">
        <h2>カメラの映像（キャリブレーション中は、ここをクリックして角を指定）</h2>
        <img id="cam" class="eizo" src="/stream" alt="camera">
        <div class="hanrei">
          <span><i style="background:#00e6ff"></i>点群（見つけたライン）</span>
          <span><i style="background:#ff00ff"></i>目標点</span>
          <span><i style="background:#00e6ff;height:3px;vertical-align:3px"></i>先読みの円</span>
          <span><i style="background:#0080ff"></i>今の方式の近くの帯</span>
        </div>
        <div class="row"><button id="shirokuro" class="b-gray">白黒で見る（ラインと判定した所が白）</button></div>
        <div class="hint">白黒にすると、しきい値の効き方がよく分かります。ラインだけが白く、床や照明の映り込みが黒くなるように「ラインのしきい値」を合わせてください</div>
      </div>
      <div class="card">
        <h2>真上から見た図（マス目は10cm。下の四角が車、前が上）</h2>
        <img id="tori" class="eizo" alt="top view" style="display:none; max-width:520px; margin:0 auto">
        <div id="tori-nashi">キャリブレーションが済むと、ここに床を真上から見た図が出ます。<br>
          右の「キャリブレーション」の手順どおりに進めてください。</div>
        <div class="hanrei">
          <span><i style="background:#ff00ff"></i>目標点と、そこへ向かう円弧</span>
          <span><i style="background:#ffc800"></i>キャリブレーションに使った四角形</span>
        </div>
      </div>
    </div>

    <div class="col">
      <div class="card">
        <h2>いまの計算結果</h2>
        <div class="vals">
          <div><div class="v" id="v-kazu">--</div><div class="k">点の数</div></div>
          <div><div class="v" id="v-todoita">--</div><div class="k">見えている距離 [m]</div></div>
          <div><div class="v" id="v-ld">--</div><div class="k">先読み距離 [m]</div></div>
        </div>
        <div class="hikaku">
          <div><div class="v" id="v-pd">--</div><div class="k">今の方式のハンドル [%]</div></div>
          <div class="pp"><div class="v" id="v-pp">--</div><div class="k">ピュアパシュートのハンドル [%]</div>
            <div class="k" id="v-kakudo">--</div></div>
        </div>
        <div class="chui" id="chui"></div>
        <div class="hint">ハンドルは本番と同じ表し方（±1.00 が左右いっぱい）。このプログラムは計算するだけで、ハンドルもモーターも動きません</div>
      </div>

      <div class="card">
        <h2>キャリブレーション（最初に1回。カメラの向きを変えたらやり直す）</h2>
        <ol class="tejun">
          <li>床にテープで四角形を作る（横40cm×奥行き60cmくらい。大きいほど正確）</li>
          <li>四角形の左右の中心を車の中心線に合わせ、車をまっすぐ向けて置く</li>
          <li>後輪の車軸から、四角形の手前の辺までの距離を測る</li>
          <li>「開始」を押して映像を止め、4つの角をクリック（順番は自由）</li>
          <li>寸法を入れて「計算して保存」</li>
        </ol>
        <div class="sunpo">
          <label>四角形の横幅 [cm]<input id="s-haba" type="number" step="0.5" value="40"></label>
          <label>四角形の奥行き [cm]<input id="s-oku" type="number" step="0.5" value="60"></label>
          <label>車軸から手前の辺まで [cm]<input id="s-kyori" type="number" step="0.5" value="45"></label>
          <label>中心の左へのずれ [cm]（ふつう0）<input id="s-yoko" type="number" step="0.5" value="0"></label>
        </div>
        <div class="row">
          <button id="c-hajime" class="b-acc">開始（映像を止める）</button>
          <button id="c-hozon" class="b-ok" disabled>計算して保存</button>
          <button id="c-yame" class="b-gray" disabled>やめる</button>
        </div>
        <div class="hint" id="c-joutai">--</div>
      </div>

      <div class="card">
        <h2>値の調整（動かすとすぐ映像と図に反映。気に入ったら保存）</h2>
        <div class="chosei" id="chosei"></div>
        <div class="chosei">
          <div class="nm">想定するアクセル [%]（保存されない。先読み距離の伸びを見る用）</div>
          <div class="at" id="at-SOTEI_ACCEL">--</div>
          <input type="range" id="sl-SOTEI_ACCEL" min="0" max="100" step="1">
        </div>
        <div class="row"><button id="hozon" class="b-ok">値を保存する</button></div>
        <div class="hint" id="h-sokudo">--</div>
      </div>

      <div class="card">
        <h2>本番（rc_race.py）で使う方式</h2>
        <div class="row" style="margin-top:0">
          <button id="a-pd" class="b-gray">今の方式</button>
          <button id="a-pp" class="b-gray">ピュアパシュート</button>
        </div>
        <div class="hint">rc_race.py の画面でも切り替えられます。ピュアパシュートを選んでも、目標点が決まらないコマは自動で今の方式に戻ります。<b>初めて切り替えたときは、車輪を浮かせてハンドルの向きを確かめてから走らせてください</b></div>
      </div>
    </div>
  </div>

<script>
  const $ = id => document.getElementById(id);
  const post = (path, data) => fetch(path, {method:"POST",
    headers:{"Content-Type":"application/json"}, body: JSON.stringify(data || {})});
  const CHOSEI = __CHOSEI__;
  const KIZAMI = Object.fromEntries(CHOSEI.map(c => [c[0], c[4]]));
  const keta = (n, v) => Number(v).toFixed((KIZAMI[n] ?? 0.01) >= 1 ? 0 : 2);
  let sawatteru = null;            // いま動かしているスライダー（動かしている間は上書きしない）

  // 調整スライダーを作る
  CHOSEI.forEach(([n, setsumei, saisho, saidai, kizami]) => {
    $("chosei").insertAdjacentHTML("beforeend",
      `<div class="nm">${setsumei}</div><div class="at" id="at-${n}">--</div>
       <input type="range" id="sl-${n}" min="${saisho}" max="${saidai}" step="${kizami}">`);
  });
  [...CHOSEI.map(c => c[0]), "SOTEI_ACCEL"].forEach(n => {
    const sl = $("sl-" + n);
    sl.oninput = () => { sawatteru = n; $("at-" + n).textContent = keta(n, sl.value);
                         post("/atai", {namae: n, atai: Number(sl.value)}); };
    sl.onchange = () => { sawatteru = null; };
  });

  $("cam").onclick = ev => {
    const r = $("cam").getBoundingClientRect();
    post("/click", {x: (ev.clientX - r.left) / r.width, y: (ev.clientY - r.top) / r.height});
  };
  $("c-hajime").onclick = () => post("/calib_hajime");
  $("c-yame").onclick = () => post("/calib_yame");
  $("c-hozon").onclick = () => post("/calib_hozon", {
    haba: Number($("s-haba").value) / 100, okuyuki: Number($("s-oku").value) / 100,
    kyori: Number($("s-kyori").value) / 100, yoko: Number($("s-yoko").value) / 100});
  $("hozon").onclick = () => post("/hozon");
  $("shirokuro").onclick = () => post("/shirokuro");
  $("a-pd").onclick = () => post("/algo", {algo: "pd"});
  $("a-pp").onclick = () => post("/algo", {algo: "pp"});

  let toriHajimeta = false, sunpoIreta = false;
  const f2 = v => (v === null || v === undefined) ? "--" : Number(v).toFixed(2);
  const fugo = v => (v === null || v === undefined) ? "--" : (v >= 0 ? "+" : "") + Number(v).toFixed(2);
  async function poll() {
    try {
      const s = await (await fetch("/status")).json();
      $("mode").textContent = s.calib_zumi
        ? `キャリブレーション済み（見える範囲: 車軸から ${f2(s.mieru_chikai)} m 〜 ${s.mieru_tooi === null ? "地平線" : f2(s.mieru_tooi) + " m"}） ・ ${s.fps} fps`
        : `キャリブレーションがまだです ・ ${s.fps} fps`;
      $("mode").className = s.calib_zumi ? "ok" : "";
      $("msg").textContent = s.message || "";
      $("cam").className = "eizo" + (s.calib_chu ? " click" : "");
      if (s.calib_zumi && !toriHajimeta) { $("tori").src = "/tori"; toriHajimeta = true; }
      $("tori").style.display = s.calib_zumi ? "block" : "none";
      $("tori-nashi").style.display = s.calib_zumi ? "none" : "block";
      if (s.shikaku && !sunpoIreta) {          // 前回の寸法を入れておく
        $("s-haba").value = s.shikaku[0] * 100; $("s-oku").value = s.shikaku[1] * 100;
        $("s-kyori").value = s.shikaku[2] * 100; $("s-yoko").value = s.shikaku[3] * 100;
        sunpoIreta = true;
      }
      $("v-kazu").textContent = s.calib_zumi ? s.kazu : "--";
      $("v-todoita").textContent = f2(s.todoita);
      $("v-ld").textContent = f2(s.ld);
      $("v-pd").textContent = s.line_mieru ? fugo(s.handoru_pd) : "ライン無し";
      $("v-pp").textContent = !s.calib_zumi ? "--" : (s.pp_ok ? fugo(s.handoru_pp) : "目標点無し");
      $("v-kakudo").textContent = s.pp_ok
        ? `切れ角 ${fugo(s.kakudo)}° ・ 目標点 前${f2(s.mokuhyo[0])}m 左${fugo(s.mokuhyo[1])}m`
          + (s.gen < 1 ? ` ・ アクセル×${f2(s.gen)}` : "")
        : "--";
      let chui = "";
      if (s.calib_zumi && s.pp_ok && s.todoita !== null && s.todoita < s.ld - 0.03)
        chui = `ラインが ${f2(s.todoita)}m 先までしか見えておらず、先読み距離 ${f2(s.ld)}m に届いていません。` +
               "カメラをもう少し上向きにするか、先読み距離を短くしてください（届かない間は、見えている一番遠い点を狙います）";
      else if (s.calib_zumi && s.line_mieru && !s.pp_ok)
        chui = "ラインは見えていますが、目標点が決まりません。この状態では本番は今の方式で走ります";
      $("chui").textContent = chui;
      $("chui").style.display = chui ? "block" : "none";

      $("c-hajime").disabled = s.calib_chu;
      $("c-hozon").disabled = !(s.calib_chu && s.kado === 4);
      $("c-yame").disabled = !s.calib_chu;
      $("c-joutai").textContent = s.calib_chu ? `クリックした角: ${s.kado} / 4（5回目を押すと最初からやり直し）`
        : (s.calib_zumi ? "保存済み。やり直すときは「開始」から" : "まだ保存されていません");

      Object.entries(s.atai).forEach(([n, v]) => {
        if (sawatteru === n) return;
        $("sl-" + n).value = v; $("at-" + n).textContent = keta(n, v);
      });
      if (sawatteru !== "SOTEI_ACCEL") {
        $("sl-SOTEI_ACCEL").value = s.sotei_accel; $("at-SOTEI_ACCEL").textContent = s.sotei_accel.toFixed(0);
      }
      $("hozon").textContent = s.mihozon ? "値を保存する（まだ保存していません）" : "値を保存する";
      $("hozon").className = s.mihozon ? "b-ok" : "b-gray";
      $("h-sokudo").textContent = s.atai.PP_SOKUDO_KEISU > 0
        ? `アクセル ${s.sotei_accel.toFixed(0)}% のとき、速度を ${f2(s.sokudo)} m/s と見積もり、先読み距離は ${f2(s.ld)} m`
        : "「アクセル1%あたりの速度」が 0 の間は、先読み距離は基本の値で固定です";
      $("shirokuro").textContent = s.shirokuro ? "カメラの映像に戻す" : "白黒で見る（ラインと判定した所が白）";
      $("shirokuro").className = s.shirokuro ? "b-sel" : "b-gray";
      $("a-pd").className = s.algo === "pd" ? "b-sel" : "b-gray";
      $("a-pp").className = s.algo === "pp" ? "b-sel" : "b-gray";
      $("a-pp").disabled = !s.calib_zumi;
    } catch (e) { /* 通信断は次のポーリングで復帰 */ }
  }
  setInterval(poll, 300);
  poll();
</script>
</body>
</html>
"""


def handler_wo_tsukuru(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _okuru(self, body, shurui):
            self.send_response(200)
            self.send_header("Content-Type", shurui)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _nagasu(self, toridasu):
            """映像を流し続ける（MJPEG）"""
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            kankaku = 1.0 / HAISHIN_FPS
            try:
                while app.ugoiteru:
                    with app.lock:
                        jpeg = toridasu()
                    if jpeg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(kankaku)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == "/":
                html = GAMEN_HTML.replace("__CHOSEI__", json.dumps(CHOSEI, ensure_ascii=False))
                self._okuru(html.encode("utf-8"), "text/html; charset=utf-8")
            elif self.path == "/status":
                self._okuru(app.joutai_json().encode("utf-8"), "application/json")
            elif self.path == "/stream":
                self._nagasu(lambda: app.cam_jpeg)
            elif self.path == "/tori":
                self._nagasu(lambda: app.tori_jpeg)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, TypeError):
                d = {}
            try:
                if self.path == "/click":
                    app.click(float(d.get("x", 0)), float(d.get("y", 0)))
                elif self.path == "/calib_hajime":
                    app.calib_hajime()
                elif self.path == "/calib_yame":
                    app.calib_yame()
                elif self.path == "/calib_hozon":
                    app.calib_hozon(float(d.get("haba", 0)), float(d.get("okuyuki", 0)),
                                    float(d.get("kyori", 0)), float(d.get("yoko", 0)))
                elif self.path == "/atai":
                    app.atai_kaeru(str(d.get("namae", "")), d.get("atai", 0))
                elif self.path == "/hozon":
                    app.atai_hozon()
                elif self.path == "/algo":
                    app.algo_erabu(str(d.get("algo", "")))
                elif self.path == "/shirokuro":
                    app.shirokuro_kirikae()
                else:
                    self.send_response(404)
                    self.end_headers()
                    return
            except (ValueError, TypeError):
                pass
            self.send_response(200)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler


def main():
    app = PpTestApp()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler_wo_tsukuru(app))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[web] ブラウザで http://<この車のIPアドレス>:{HTTP_PORT} を開いてください")
    print("[web] このプログラムはハンドルもモーターも動かしません（計算と表示だけ）")

    def tomeru(*_):
        print("\n[main] 停止処理中...")
        app.ugoiteru = False

    signal.signal(signal.SIGINT, tomeru)
    signal.signal(signal.SIGTERM, tomeru)

    app.mawasu()
    server.shutdown()
    print("[main] 終了しました")


if __name__ == "__main__":
    main()
