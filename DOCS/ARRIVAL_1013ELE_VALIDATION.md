# 1013ele退出前の地図切替検証（2026-11-15）

反映先は `/home/koko-chan-nuc/kuwa_ws2`。操作手順は同ディレクトリの `ARRIVAL_1013ELE.md`。
IMU停止・停止後OPEN → 1013ele読込 → かご面積重心への初期位置設定 → 新しいAMCL/TF・到着側OPEN・全車体かご内・車体停止を2秒確認 → 両コストマップ消去 → ドア外1.2mへの自律退出。
地図切替後の完了通知を受けた1018 waypoint経路は終了し、古い地図上の次の目標やループを送信しない。

成功bag `20261115_165330` の退出承認直前のAMCLを利用した検証では、ドア相対位置をそのまま移すと既存の到着側かごポリゴンから車体がはみ出した。このため1013ele既存かごポリゴンの面積重心（約44.944, 7.962）をAMCL初期推定に使い、ドア相対の向きを引き継ぐ。これは実測位置の保証ではない。AMCL/TFの整合と全車体かご内の再確認が必要。

検証結果:

- elevator_space_navigation・elevator_imu_monitorのビルド成功。インストール先から新モジュールとwaypoint終了処理を読み込めることを確認。
- 配置後の設定チェック成功。GUIをオフスクリーンで構築し、自動退出設定と画面構築を確認。制御プロセス起動なし。
- 到着側の初期位置中心・退出目標中心は地図画像の空きセル。経路全体や実際の障害物の通行可能性は未検証。
- arrival map、既存post-exit map、lift cycle、motion watchdog、transfer、waypoint elevator taskの選択したテストを実行し186件成功。
- 既存 `test_mux_config_uses_dedicated_persistent_lock` 1件はZIPから欠けた `src/elevator_space_navigation/config/elevator_twist_mux.yaml` を参照するためFileNotFoundError。今回の新規テストの失敗ではない。使用中の実機muxは別の設定パスで、起動時の既存preflightの対象。
- 新規arrival mapテストはROSグラフ・実機を起動せず、非同期サービス順序、地図失敗、IMU/停止根拠喪失、古いドアコールバックの排除、新しいAMCL/TF/OPENの必須条件を検証。

実機ノードの起動、走行、実際の地図切替はこの変更作業では実施していない。GUI/制御再起動後の現地確認が残る。
反映前バックアップ: `~/kuwa_ws2/backups/before_arrival_1013ele_20261115_184341`。
