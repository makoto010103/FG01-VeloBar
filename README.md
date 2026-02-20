# FG01-VeloBar — Velocity Based Training Device

Seeed XIAO nRF52840 と ICM-42688 IMU を使用した、バーベル挙上速度をリアルタイムで計測するVBTデバイスです。  
BLE（Bluetooth Low Energy）でスマートフォンに接続し、Webアプリ上でリアルタイムの速度グラフ・レップ検出・疲労度管理を行います。

> **現在のバージョン: v3.2.1**

---

## 📱 Webアプリ（GitHub Pages）

GitHub Pagesで公開された `index.html` がそのままトレーニングアプリとして動作します。

### 主な機能
- ⚡ **リアルタイム速度計測** — BLE接続でピーク速度を即座に表示
- 📊 **ライブグラフ** — 12秒ウィンドウの速度波形をリアルタイム描画
- 🏋️ **種目・重量選択** — SQ / DL / BP / CL に対応、種目別の速度閾値を自動設定
- 🎯 **VBT Judgment Engine** — パワー / 筋力 / 筋肥大モードで Velocity Loss を自動計算
- 🔴 **波形記録 & CSVエクスポート** — 5秒カウントダウン付きの波形録画、クリップボードコピー
- 🔊 **音声フィードバック** — ベル / ゲーム / 電子音 / 機械音 4種類から選択

### 種目別 最小速度閾値（Min Peak Velocity）
| 種目 | 閾値 (m/s) | 備考 |
|------|-----------|------|
| BP (ベンチプレス) | 0.25 | |
| SQ (スクワット) | 0.35 | |
| DL (デッドリフト) | 0.30 | |
| CL (クリーン) | 0.50 | |

---

## 🔧 ハードウェア

### 部品リスト
| 部品 | 型番 | 備考 |
|------|------|------|
| マイコン | Seeed XIAO nRF52840 | BLE内蔵 |
| IMU | ICM-42688 (Strawberry Linux製モジュール) | 6軸（加速度+ジャイロ） |

### 配線
詳細は [`docs/wiring_diagram.md`](docs/wiring_diagram.md) を参照してください。

| ICM-42688 | XIAO nRF52840 | 役割 |
|-----------|---------------|------|
| VDD | 3V3 | メイン電源 |
| VDDIO | D1 | I/O電源（GPIO制御） |
| SCL | D5 | I2Cクロック |
| SDA | D4 | I2Cデータ |
| AD0 | D2 | アドレス設定（LOW=0x68） |
| ~CS | D3 | I2Cモード（HIGH） |
| GND | GND | グラウンド |

---

## 🚀 セットアップ

### 1. ファームウェア書き込み
1. [Arduino IDE](https://www.arduino.cc/en/software) をインストール
2. ボードマネージャで **Seeed nRF52 Boards** を追加  
   URL: `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
3. 以下のライブラリをインストール:
   - `Adafruit ICM42688` (IMUドライバ)
   - `MadgwickAHRS` (姿勢推定フィルター)
4. `firmware/vbt_firmware/vbt_firmware.ino` をマイコンに書き込み

### 2. Webアプリの公開
1. このリポジトリをGitHubにPush
2. Settings > Pages > Source: **Deploy from a branch** / Branch: **main** / Folder: **/ (root)**
3. 発行されたURLにスマホでアクセス → VBTダッシュボードが起動

### 3. 使い方
1. Webアプリを開き、**⚡ 接続** ボタンをタップ
2. 「VeloBar」を選択してBLE接続
3. 種目・重量・モードを設定してトレーニング開始
4. 波形を記録する場合は **🔴 波形記録** → 5秒カウントダウン後に記録開始

---

## 📁 プロジェクト構成

```
vbt_project/
├── index.html                    # Webダッシュボード（GitHub Pages）
├── firmware/
│   └── vbt_firmware/
│       └── vbt_firmware.ino      # マイコン用ファームウェア
├── docs/
│   ├── CHANGELOG.md              # バージョン履歴
│   └── wiring_diagram.md         # 配線図
├── research/                     # 開発時のテスト・調査用プログラム
├── .agent/
│   └── workflows/
│       └── versioning.md         # バージョン管理ルール
└── README.md                     # このファイル
```

---

## 📈 ファームウェア技術詳細

### 速度計測の仕組み
1. **IMUから加速度・ジャイロ取得**（~200Hz）
2. **Madgwickフィルター**で姿勢（クォータニオン）を推定
3. 推定した重力方向を除去して**鉛直方向の加速度**を抽出
4. 加速度を**時間積分**して速度を算出
5. **Zero Velocity Update (ZUPT)** で静止時のドリフトを補正
6. BLEで50Hz（20ms間隔）でスマートフォンに送信

### 主要パラメータ
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| 加速度レンジ | ±16G | 高重量のバーベル動作に対応 |
| ジャイロレンジ | ±2000 dps | 高速回転検出用 |
| BLE送信レート | 50Hz | 20ms間隔 |
| ZUPT閾値 | ジャイロ < 5dps & 加速度 ≒ 1G | 25フレーム継続で静止判定 |
| 回転クランプ | > 300 dps | バーベル回転によるノイズ除去 |

---

## 📋 変更履歴

詳細は [`docs/CHANGELOG.md`](docs/CHANGELOG.md) を参照してください。
