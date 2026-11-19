# 最新bagと同時刻ログ：ウェイポイント途中終了

解析日：2026-11-15。時刻は日本時間。実機起動・bag再生・設定変更は行っていません。

## 対象と制約

最新のmetadata更新時刻を持つ記録は `/home/koko-chan-nuc/kuwa_ws2/records/20261115_190632`。
記録時間は約318秒、19:06:34頃〜19:11:52頃です。
このbagには `/rosout`、Nav2 action status/feedback、global/local costmap、
`/cmd_vel_nav` は含まれません。中断の直接理由は、同時刻のROSログで照合しました。
`/home/koko-chan-nuc/rosbag2_data/1115` は17:03開始の古い別記録です。

## 19:08の中断

1. 19:07:13.205 WP7が距離0.99mで通過判定。
2. 19:08:05.329 BT navigatorが `compute_path_to_pose` の目標受付応答待ちタイムアウトを記録。
3. 19:08:05.404 Nav2がabort、19:08:05.459追従側がWP8失敗としてスキップ。
4. 19:08:06.541 WP9でも同じタイムアウト、19:08:06.610失敗。
5. 19:08:06.681 WP10でも同じタイムアウト、19:08:06.695失敗。

したがって、ウェイポイント配列が途中で切れたのではなく、Nav2中断を受けて
後続点をスキップし、最後の点まで失敗して終了しています。

## 19:09の再実行

同じ `1018.yaml` の10点をWP5から再実行しています。

1. 19:09:20.370 位置(0.78, 3.08)からWP5 (3.35, -1.05)へ開始。
2. 19:09:20.425〜19:09:40.538、plannerが繰り返し
   `Failed to create plan with tolerance of: 0.500000`。
   この試行の最初の失敗は、WP5への経路生成失敗です。
3. 19:09:40.561 WP5 abort、失敗点をスキップしてWP6へ。
4. 19:09:46.859 WP6で `compute_path_to_pose` 受付応答待ちタイムアウト。
5. 19:09:46.909〜19:09:47.022、WP7〜WP10も同じタイムアウト。
   WP6中断からWP10中断まで約0.15秒です。

WP5、WP6の目標セル自体はbagの静的地図ではoccupancy=0でした。
ただし静的地図上で目標セルが空いていても、動的障害物・膨張・keepoutを含む
当時のglobal costmapで到達可能だったことは示せません。
このbagにcostmapがないため、WP5の経路が作れなかった具体的な障害物は未確定です。

## 終了を早める設定と確認すべき点

- 追従ログの実効設定は `stop_on_failure=False`。
  `waypoint_follow.py` の `_handle_failure` は通常点で失敗すると次点へ進みます。
  一方、最後のWP10は `elevator: true` のため失敗時に追従を終了します。
- 現在の `params/real/nav2.yaml` は `bt_navigator.default_server_timeout: 20`。
  ローカルのNav2実装で、この応答待ちは `std::chrono::milliseconds` 型です。
  20秒ではなく20ミリ秒なので、応答遅延に対して非常に短い設定です。
  ただしbagにパラメータの実効値記録がなく、当時の実効値までは未確認です。
- controllerログには19:06〜19:08の `Optimizer fail to compute path`、
  `Failed to make progress`、制御周期遅延もあります。
  CPU負荷・DDS遅延・costmap処理のどれが受付遅延を生んだかは未確定です。
- bag上のエレベータ側は中断時に `WAIT_START` / stop_latched=false。
  `MANUAL_STOP` と `/elevator_safety_stop=True` は19:10:30.627で、
  上記のNav2中断より後です。エレベータ安全停止が最初の中断原因ではありません。

次の切り分けでは、実効タイムアウト値を確認して応答待ちの調整を検討し、
`stop_on_failure=true` で最初の失敗点を保持して調べるのが有用です。
再記録には `/rosout`、Nav2 action status/feedback、global/local costmap、
速度指令の各段階、パラメータのスナップショットを含める必要があります。
設定はこの解析では変更していません。

## 根拠ログ

- `/home/koko-chan-nuc/.ros/log/python3_180001_1794737146297.log`
- `/home/koko-chan-nuc/.ros/log/python3_185085_1794737354937.log`
- `/home/koko-chan-nuc/.ros/log/bt_navigator_178431_1794737029320.log`
- `/home/koko-chan-nuc/.ros/log/planner_server_178429_1794737029328.log`
- `/home/koko-chan-nuc/.ros/log/controller_server_178427_1794737029341.log`
