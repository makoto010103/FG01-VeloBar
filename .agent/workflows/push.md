---
description: コード変更後に毎回 git add / commit / push を実行する
---

// turbo-all

1. 変更をステージング
```
cd /Users/itoumakoto/Antigravity/vbt_project && git add -A
```

2. コミット（バージョン番号と変更内容を含むメッセージ）
```
cd /Users/itoumakoto/Antigravity/vbt_project && git commit -m "<バージョン>: <変更内容の日本語要約>"
```

3. リモートへプッシュ
```
cd /Users/itoumakoto/Antigravity/vbt_project && git push
```
