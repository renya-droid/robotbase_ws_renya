# 2026-11-15 直近2回の入退室試験

解析対象はkuwa_ws2/recordsの直近2回。元bagとROSログは読み取りのみで、制御・設定の変更や実機起動はしていません。時刻は日本時間です。

| 記録 | 結果 | 最終状態 |
|---|---|---|
| `/home/koko-chan-nuc/kuwa_ws2/records/20261115_165330` | 入室→昇降停止検出→自動退出→車体全体の退出・停止確認まで完了 | FINISHED / lift_cycle=COMPLETE / GOAL_REACHED |
| `/home/koko-chan-nuc/kuwa_ws2/records/20261115_170214` | 入室中のNav2中断で停止。昇降連携・自動退出まで未到達 | STOP_LATCHED / NAV_TERMINAL_FAILURE 6 / NG_NAV_FAILED |

## 1回目：完了を確認

- 16:57:40.585 入室目標送信。
- 16:57:51.638 入室Nav2目標成功（result_status=4）。
- 16:57:53頃 乗車・車体停止確認後の閉扉待ち。
- 16:58:00.627 IMU下降加速推定。
- 16:58:04.628 IMU下降減速推定。
- 16:58:09.627 IMU停止推定。
- 16:58:10.652 automatic_exit_authorized。source=imu_stop_and_fresh_open。人の承認なしで退出許可。
- 16:58:15頃 EXITING。
- 16:58:30.401 退出Nav2目標成功（result_status=4）、doorway_region=OUTSIDE。
- 16:58:31.151 whole_body_exit_stopped。
- 16:58:31頃 FINISHED、lift_cycle=COMPLETE、waypoint_cycle_result=GOAL_REACHED。
- 最終状態はnav_unresolved=false、normal_profile_ready=true、map_switched=false。

ユーザーの「1回成功した」という報告と、制御ログ上の一連動作完了が一致しています。地図切替は無効のままです。

## 2回目：入室中のNav2中断

- 17:04:47.638頃 Nav2開始。現在位置(2.01, 10.05)、目標(-0.32, 11.24)。
- 17:04:52.978 controller_serverに制御周期遅延の警告。設定10Hzに対してこの時点で3.4267Hz。
- 17:04:53.049 bt_navigator: `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`。
- 17:04:53.073 bt_navigatorがnavigate_to_poseをabort。
- 17:04:53.093 入退室側がNav2結果6（STATUS_ABORTED）を受信。
- 17:04:53.117 NAV_TERMINAL_FAILURE 6で停止要求。

中断時、ドアはOPEN、door_source_valid=true、scan/poseの入力監視はOKでした。車体領域の推定はSTRADDLING。
従って、この中断はドア判定停止やIMU停止待ちの問題ではなく、Nav2のcompute_path_to_pose目標要求への応答待ちタイムアウトが直接の中断理由です。
制御周期遅延も同時刻にありますが、その根本原因をCPU負荷等と断定するには、この記録だけでは不十分です。

その後17:05:23.931からIMUは上昇加速・減速・17:05:32.934停止を推定しています。しかし入退室側はすでに停止保持で、lift_cycle=IDLEのため、自動退出へは進みません。

確認ログ：

- `/home/koko-chan-nuc/.ros/log/bt_navigator_99328_1794729600596.log`
- `/home/koko-chan-nuc/.ros/log/controller_server_99324_1794729600633.log`
- `/home/koko-chan-nuc/.ros/log/planner_server_99326_1794729600615.log`

## 研究結果としての範囲

両方とも記録のresearch.methodはlegacyで、sweep_monitor.enabled=false、record_only=falseです。
今回の成功は、記録された従来法構成での1回の一連動作完了として扱います。提案手法の成功や、反復試験による安定性を確認したものではありません。
