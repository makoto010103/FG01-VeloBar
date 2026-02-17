# VBTデバイス セットアップ＆操作マニュアル

## 1. ハードウェアの接続
XIAO nRF52840とICM-42688センサーを以下の通りに接続します。（詳細は [wiring_diagram.md](file:///Users/itoumakoto/Antigravity/vbt_project/docs/wiring_diagram.md) を参照）

- **3V3** <-> **VDD**
- **D1** <-> **VDDIO**
- **D5** <-> **SCL**
- **D4** <-> **SDA**
- **D2** <-> **AD0**
- **D3** <-> **~CS**
- **GND** <-> **GND**

## 2. ソフトウェアの書き込み
Arduino IDEで以下のファイルを開き、マイコンボードへ書き込んでください。
- **ファイル**: [vbt_firmware.ino](file:///Users/itoumakoto/Antigravity/vbt_project/firmware/vbt_firmware/vbt_firmware.ino)

## 3. アプリとの接続と計測

### 方法A：専用Webダッシュボード（おすすめ）
「数字の表示」と「ログの保存」が同時にできる専用アプリです。
1. スマートフォンのブラウザで **[index.html](file:///Users/itoumakoto/Antigravity/vbt_project/index.html)** を開きます。
2. **[Connect Device]** ボタンを押し、`VBT_Device` を選択します。
3. **リアルタイム表示**: 速度が巨大なフォントで画面に表示されます。
4. **ログ・グラフ**: 自動で1レップごとの最高速度が記録され、グラフも描画されます。
5. **保存**: [Download CSV] ボタンで、計測データをExcel等で開ける形式で保存できます。

> [!NOTE]
> **iOS (iPhone/iPad) をお使いの方へ**
> 標準のSafariはBluetooth通信を制限しているため、App Storeから **「Bluefy」** または **「WebBLE」** という無料ブラウザアプリをインストールし、そのアプリ内で上記HTMLファイルを開いてください。AndroidのChromeはそのまま動作します。

### 方法B：nRF Connect アプリ（詳細確認用）
通信状態を詳しく確認したい場合に。
1. **nRF Connect** アプリで **`VBT_Device`** に Connect します。
2. UUID **`180C`** > **`2A6E`** の右にある矢印アイコン（Notify）をタップします。

## 4. トレーニングでの使い方（ここが重要！）
1. **電源オン**: USBバッテリーなどに繋ぎ、電源を入れます。
2. **キャリブレーション（Zeroing）**:
   - 電源を入れた直後の **1〜2秒間は絶対に動かさないでください。**
   - この間に、その場の傾きを自動で検知し「0点」を設定します。
3. **リフト開始**:
   - 通常通りリフト動作を行います。
   - スマホ画面にリアルタイムで「举上速度 (m/s)」が表示されます。
   - バーベルをラックに戻して静止すると、数値は自動的に **0.00** に戻ります。

## トラブルシューティング
- **数字がズレる**: リセットボタンを押して、もう一度平らな場所でキャリブレーション（静止）させてください。
- **接続できない**: スマホのBluetoothを一度OFF/ONしてみてください。
