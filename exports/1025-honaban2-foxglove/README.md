# 1025 本番2 — rosbag + 外部動画

## 開き方

1. Foxglove の「Open local file」で `1025-honban2-with-video.mcap` を開く。
2. Layouts → Import from file で `foxglove-layout.json` を読み込む。
3. 再生すると、左に地図・LiDAR・自己位置、右に外部動画と速度グラフを表示する。
4. **動画は rosbag の先頭から約7分5.3秒後に始まる。** 最初は画像が表示されないため、その付近までシークする。

レイアウトのインポートが使えない場合は、Image パネルを追加し、Topic に
`/external_video/image/compressed` を指定する。
3D パネルは Fixed frame を `map`、Follow frame を `robot/base_footprint` とし、
`/map`、`/scan3`、`/amcl_pose`、`/plan` を表示する。
再生・一時停止・シークは映像とrosbagで共通。

## 同期時刻（日本時間）

- rosbag: 2026-10-25 13:55:27.700 〜 14:13:01.760（約17分34秒）
- 動画: 2026-10-25 14:02:33 〜 14:11:44.267（約9分11秒）
- 動画の開始位置: rosbag 開始から425.300秒後

同期には MOV 内の `com.apple.quicktime.creationdate`（撮影日時）を使った。
ファイル作成日時・コピー日時は使っていない。
**PCとiPhoneの時計差は未校正なので、実際の動きとの厳密な同期は未確認。**
動画のない最初の約7分と最後の約1分17秒には新しい映像がない。

## データ

- 主ファイル: `1025-honban2-with-video.mcap`（rosbag 全メッセージ + 動画）
- rosbagのみ: `1025-honaban2-foxglove.mcap`
- 動画のみ: `1025-honban2-video.mcap`
- レイアウト: `foxglove-layout.json`（通常はこちらを使用）
- SDK形式のレイアウト: `foxglove-layout-v1.json`
- 検証結果: `conversion-report.json`、`video-sync-report.json`、`merged-validation.json`

動画は1280×720・10fpsのJPEG画像トピックへ変換し、HDRをSDRに変換した。
元のMOVに音声トラックはない。元のrosbagとMOVは変更していない。
rosbagの欠落していたメッセージ定義とインデックスを再構築した。
元のrosbag全390,951メッセージについて、トピックごとの件数、本文、
記録時刻・送信時刻・sequenceのSHA256一致を確認済み。
統合ファイルも件数・動画の先頭末尾・MCAP構造検査を通過（警告・エラー0件）。
Foxgloveアプリ画面での表示確認は未実施。
生成されたROS actionのフィードバック型は、Foxgloveのデコーダーが読める
メッセージ型名へ別名を付けている（本文は変更していない）。

Foxglove公式資料: https://docs.foxglove.dev/docs/visualization/layouts
