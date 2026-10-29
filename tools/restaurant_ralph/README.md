# Ralph実行前レビュー

Loopはまだ起動していません。RALPH_PROMPT.mdを入力とし、EVALUATION.mdのG0〜G7全合格を完了条件にします。BASELINE.mdはコード調査、PROGRESS.mdは反復引継ぎです。

準備済みworktree: `/tmp/restaurant-ralph-research-20261029`
branch: `ralph/restaurant-research-20261029`
元HEAD: `36cc7c3`。元workspaceでのcommit/stash/resetは行っていません。準備文書は両方へコピーした未コミットファイルです。元画像はworktree内referenceへコピーしSHA256をmanifestに記録しました。/tmpはOSの清掃対象なので長期保管は別の永続パスへ `git worktree move` し、manifestのworktree/reference_imageも更新してください。作業後は成果をレビューしてから保存・統合します。

## 起動前の確認（Loopを実行しない）

```bash
cd /tmp/restaurant-ralph-research-20261029
bash tools/restaurant_ralph/run.sh
bash tools/restaurant_ralph/baseline.sh
```

## 内容レビュー後の推奨実行

新しいターミナルを開き、元workspaceのinstallをsourceせずに実行します。

```bash
cd /tmp/restaurant-ralph-research-20261029
bash tools/restaurant_ralph/run.sh --run
```

最大15反復。実際のralphコマンドはrun.sh内にあり、Codex、workspace-write、no-auto-commit、完了/中止promiseを指定。--no-allow-allにより権限の無条件許可を避けます。非対話実行で権限が不足したら失敗を記録する方針です。sandbox初期化エラーなどは権限無効化で回避せず環境問題として報告します。Codex認証・実際のエージェント起動は今回未確認です。反復上限到達は完了ではありません。

停止は実行ターミナルでCtrl+C。状態確認は同じworktreeで `ralph --status`。Loopの停止後も自分が起動したGazebo等のPIDを記録に従って確認してください。全ユーザーのプロセスをpkillしないでください。

このworktreeはGit履歴とソースを分離するものでOSセキュリティ境界ではありません。実機Domain54/既存sim67と分離したDomain89、GZ_PARTITIONを使用します。原本・実機アクセス禁止はAGENTS.mdとpromptへ明記しています。

現在のbaselineテストは研究品質を保証しません。全シナリオの状態・実位置・Nav2を独立評価器で検査する仕組み自体がRalphの実装対象です。一次比較は混雑度、二次比較はground-truth人位置layerの有無。センサー性能の実証と混同しないでください。

初回の調査時は元Gitがcleanでした。将来の再準備で変更があれば、勝手にcommit/stashせず別名worktreeへ対象ファイルの明示的snapshotとhashを保存するか、ユーザーの保存操作後に準備してください。本スクリプトはworktreeを再作成しません。

公式資料: https://github.com/Th0rgal/open-ralph-wiggum
導入済みCLI 1.2.2のhelpとソースを優先してコマンドを検証しています。
