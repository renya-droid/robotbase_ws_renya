# 調査結果 2026-10-29

調査開始HEAD: 36cc7c3、branch renya/camera-nav。開始時のgit statusは空（未コミット変更なし）。それでも今後の利用時はdirtyを検出して停止する。

既存目標: READMEには実機/シミュレーションNav2、MPPI、ウェイポイント走行が記述されている。レストラン固有目標は図面再現と来店・離席・給水・会計・退店を含む人混在評価。本書はコード読み取り結果であり既存READMEの実機動作を再検証したものではない。

実装: restaurant_simはPythonパッケージ、4 SDF、layout.json、3シナリオ(5/10/18人)、A*4近傍、動的再探索、非同期CLI一括pose更新、PoseArray出力。既存pytestは2件（到達性、静止人物迂回）。既存コードは直前の修正までGitへ保存済み。

再利用候補: robotbase_sim/launch/{sim,navigation}.launch.py、models、params/sim、external_camera_people_layerのPoseArray全件snapshot入力。README記載robot footprintは0.60×0.58 m（実設定と再照合）。実機Domain 54、既存sim 67と分離する。

不足/コード上の懸念:
- 永続人物ID、姿勢、速度、状態がROSから取得できない。姿勢z変更も待機開始時に送信されず表示が一致しない可能性。
- 更新失敗でも内部位置が進む。dirty辞書を残し続ける。CLI起動ごとの探索遅延。dtの0.2秒clipで実sim時間と行動時計がずれる。
- 予約/サービス列/同行者/退避戦略なし。接近停止+再探索のみで狭路待ちの解消保証なし。
- 着席は椅子横の停止。初期着席者は時差出現。推定17×11.5 m、会計・給水の用途未確認。
- 生成器削除済みでSDFとJSONが二重管理。図面overlay・地図・ロボット統合launchなし。
- mapとworldを同一とみなしTF変換しない。ロボットodomの鮮度検査なし。
- 全シナリオ完走/掃引衝突/サービス列/観測整合/Nav2研究指標の試験なし。README検証欄は修正前の内容を含み更新が必要。

Ralph CLI: 導入済み1.2.2、--prompt-file、--agent codex、--max-iterations、--no-commit、--no-allow-all、--abort-promiseをhelpで確認。公式: https://github.com/Th0rgal/open-ralph-wiggum 。ローカル実装の既定allow-allはCodexへ--full-autoを渡す。推奨は--no-allow-allと明示workspace-writeを用い、無制限反復/自動コミットは避ける。
