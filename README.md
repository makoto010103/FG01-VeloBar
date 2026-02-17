# FG01-VeloBar (Velocity Based Training Device)

Seeed XIAO nRF52840 と ICM-42688 センサーを使用した、挙上速度をリアルタイムで計測・記録するVBTデバイスのプロジェクトです。

## 🚀 プロジェクトの構成
- `index.html` : **Webダッシュボード**（GitHub Pagesでアプリとして動作します）
- `firmware/` : マイコン（Arduino）用の最新ソースコード
- `docs/` : 配線図、ユーザーマニュアル、開発記録
- `research/` : 開発過程で使用したテスト・調査用プログラム

## 📱 アプリの使い方 (GitHub Pages)
このプロジェクトをGitHubにアップロードし、Settings > Pages から `index.html` を公開することで、スマホのブラウザから直接計測アプリとして利用できます。

1. **GitHubリポジトリ**を作成し、このプロジェクトのファイルをすべてアップロード（Push）します。
2. **GitHub Pages** を有効にします（Source: Deploy from a branch, Branch: main / root）。
3. 発行されたURLにスマホでアクセスすると、VBTダッシュボードが起動します。

## 🛠️ はじめかた
1. `docs/wiring_diagram.md` を参考にハードウェアを配線します。
2. Arduino IDEで `firmware/vbt_firmware/vbt_firmware.ino` を開き、マイコンに書き込みます。
3. 詳細な操作方法は `docs/user_manual.md` を参照してください。

## 📈 進捗と詳細
- 開発の詳細は `docs/implementation_plan.md` を参照。
- 進捗管理は `docs/task.md` を参照。
