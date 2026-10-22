# 2026-10-22 12:02 JST のsim停滞

ROS Domain 67を約9秒間読み取り。操作指令・設定変更は行っていない。
生データは `observation.json`。

- `/cmd_vel_nav`: 正の前進指令は約0.083803〜0.083810m/s。角速度はほぼゼロ。
- `/cmd_vel`: 観測161件すべて前進速度0。角速度の絶対値も約0.000032rad/s以下。
- `/odom`: 位置はほぼ不変、前進速度はほぼ0。
- `/plan`: 観測中に2回更新される。終点はmap座標約(-6.646, 0.155)。
- 現在地はTF合成で約(15.032, -5.765)、yaw約-39.0度。
- `bt_navigator` ログでは7 posesのNavigateThroughPosesを開始した記録がある。
- `controller_server` ログでは約30秒間隔で `Failed to make progress`、FollowPath中断、コストマップクリア、再試行を繰り返す。

実行中のパラメータをGetParametersで読み取り:

- `/velocity_smoother.deadband_velocity = [0.1, 0, 0]`
- `/velocity_smoother.feedback = OPEN_LOOP`
- `/controller_server.FollowPath.VelocityDeadbandCritic.enabled = true`
- 同 `cost_weight = 35`、`deadband_velocities = [0.1, 0, 0]`
- `/controller_server.min_x_velocity_threshold = 0.1`
- 進捗判定は `required_movement_radius = 0.2`、`movement_time_allowance = 30`

`nav2_velocity_smoother/src/velocity_smoother.cpp` は、前進指令の絶対値がdeadband未満なら0として配信する。
観測したNav2出力はこの0.1m/s未満に留まり、最終指令が0になっている挙動と整合する。
直接の停滞要因として、Nav2が選ぶ低速指令と速度平滑化のdeadbandの組み合わせが有力。
MPPIが約0.084m/sへ収束する理由は別途調べる必要がある。
`/cmd_vel_smoothed` は今回の9秒観測に含めていないため、経路中のどのノードで0になったかの直接照合は未完了。

以前の1F-1021 bagでは、約0.2m/sの前進と約-0.15m/sの後退が最終指令・実測速度にも現れていた。
今観測している「低速指令が出るが最終指令0で動かない」状態は、その前後往復とは区別する。
