# 開発環境セットアップガイド

## 1. Arduino IDEのインストール
[Arduino公式サイト](https://www.arduino.cc/en/software)からインストールしてください。

## 2. ボードマネージャの設定
1. Settings/Preferencesを開く
2. 追加のボードマネージャURLに以下を入力：
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
3. Boards Managerで `Seeed nRF52 Boards` をインストール。

## 3. ライブラリのインストール
以下のライブラリをLibrary Managerからインストールしてください：
- **ICM42688** (by Inhwan Wee)
- **Adafruit AHRS**
- **ArduinoBLE**

## 4. 動作確認
`firmware/vbt_firmware.ino` を書き込み、シリアルモニタでデータが表示されるか確認してください。
