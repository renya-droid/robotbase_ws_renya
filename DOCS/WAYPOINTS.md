# ウェイポイント記録・追従

## 概要

SIRIUSの`move_goal.py`と位置記録ツールを参考に、ココちゃん専用の`robotbase_waypoint`パッケージを追加した。実機・シミュレーションで同じYAMLを使い、通常の中間点をNav2の`NavigateThroughPoses`へまとめて送る。停止・待機・人物探索の点で区間を分割し、その点で既存の処理を実行してから次の区間を送る。

SIRIUSから引き継いだもの:

- `format_version: '1.0'`のウェイポイントYAML
- 開始番号、ループ（中間点切替距離はto_poseのみ）
- 地点ごとの`threshold`、`stop`、`wait_time`
- 現在位置の1点保存と、一定移動距離ごとの連続記録

ココちゃん向けに変更したもの:

- 固定の`sirius3/base_footprint`を廃止し、`robot.env`のTF接頭辞を使用
- SIRIUSの固定パスを廃止し、`robotbase_ws/maps_waypoints/waypoints/`へ集約
- LLM用`/nav_control`、地図切替shell、SIRIUS固有トピックを含めない
- 通常の中間点では同じNavigateThroughPoses actionを継続し、目標の送り直しや停止指令を入れない
- 停止・待機・人物探索の地点では到着／キャンセル確認後に従来の処理へ移る
- 停止時は`/cmd_vel_direct`へゼロ速度を送り安全に停止を維持する
- 区間の計画が失敗した場合は、次の未通過点だけを再試行する。その点も失敗した場合に`stop_on_failure: false`（既定）なら通常点をスキップする。人物探索点の失敗は従来どおり停止を維持する
- YAMLの値とファイル存在を起動時に検証し、同一goalの周期的な再送を行わない

`change_map`と`rotate`は未対応。地図は先に`koko_nav2_real`または`koko_nav2_sim_map`で選択する。

## 起動順

既存地図で使う場合、先に通常どおりセンサー・自己位置推定・twist_mux・Nav2を起動し、RVizで初期姿勢を設定する。その後、別端末でウェイポイント追従を開始する。

シミュレーション:

```bash
koko_sim
koko_rviz_sim
koko_nav2_sim_map
# RVizで2D Pose Estimateを設定
koko_waypoint_follow_sim
```

実機:

```bash
koko_roboteq
koko_velodyne
koko_imu
koko_sf_real
koko_twist_mux
koko_rviz_real
koko_nav2_real
# RVizで2D Pose Estimateを設定
koko_waypoint_follow
```

追従コマンドはYAML、開始番号、走行方式、ループ有無を順に質問する。走行方式は`1: NavigateToPose`、`2: NavigateThroughPoses`で、Enterのみなら現在と同じ`2`を使う。走行方式の選択後、`1`では中間点の切替距離（既定1.0 m）、`2`では通常区間の切替距離（既定6.0 m）を質問する。選択するYAMLと、Nav2で選択した地図の座標系が一致している必要がある。

MPPIは後退より前進を強く優先する。最初の地点は、現在姿勢から見て前方に置く。後方から開始すると、到達まで大きく旋回する場合がある。

## 地点の記録

### SLAM地図とウェイポイントを同時に作る

SIRIUSの`get_position_distance.py`相当として、SLAM中の現在姿勢を一定距離ごとに記録し、終了時に地図とウェイポイントを同じ名前で保存できる。

先にSLAM Toolbox（またはSLAMとNav2の同時構成）とRVizを起動し、手動走行できる状態にする。

```bash
# 実機
koko_nav2_real_slam
koko_rviz_real
koko_keyop2
koko_slam_waypoint_record

# シミュレーション
koko_sim
koko_nav2_sim_slam
koko_rviz_sim
koko_keyop2_sim
koko_slam_waypoint_record_sim
```

記録名と距離間隔を入力して走行する。`Ctrl+C`を押すとウェイポイント記録を終了し、現在の`/map`も保存する。例えば名前を`1F-route`にすると次の3ファイルが作られる。

```text
maps_waypoints/maps/1F-route.yaml
maps_waypoints/maps/1F-route.pgm
maps_waypoints/waypoints/1F-route.yaml
```

GUIランチャーでは「地図＋ウェイポイント記録（実機）」または「地図＋ウェイポイント記録（シミュレーション）」プリセットを使える。

`map -> <prefix>/base_footprint`が得られない間は記録されない。SLAMとセンサーを先に正常起動する必要がある。

現在姿勢を1点追加する:

```bash
koko_waypoint_save       # 実機
koko_waypoint_save_sim   # シミュレーション
```

指定距離を移動するたびに記録し、`Ctrl+C`まで続ける:

```bash
koko_waypoint_record       # 実機
koko_waypoint_record_sim   # シミュレーション
```

距離記録中はキーオペ端末を選択したまま`p`キーを押せる。キーオペが
`/waypoint_record/pause`を通知し、記録ノードがその時点の姿勢へ`stop: true`と
`wait_time: 10.0`を設定する。追従時はその地点で10秒停止し、その後に次の
ウェイポイントへ自動的に進む。記録端末を選択して直接`p`を押す操作にも対応する。

同名ファイルがあれば末尾へ追記し、番号は既存の最終番号から続ける。保存中は`map -> <prefix>/base_footprint`が必要なため、AMCLまたはSLAM Toolboxを先に起動する。

## YAML形式

```yaml
format_version: '1.0'
waypoints:
- number: 1
  x: 1.0
  y: 0.0
  angle_radians: 0.0
  threshold: 0.5
- number: 2
  x: 2.0
  y: 1.0
  angle_radians: 1.571
  stop: true
  wait_time: 3.0
- number: 3
  x: 2.0
  y: 2.0
  angle_radians: 1.571
```

| キー | 必須 | 意味 |
|---|---|---|
| `number` | 省略可 | 表示上の地点番号。省略時はリスト順 |
| `x`, `y` | 必須 | `map`座標系の位置[m] |
| `angle_radians` | 必須 | 到着姿勢のyaw[rad] |
| `threshold` | 省略可 | 中間点を通過済みと判定する距離[m] |
| `stop` | 省略可 | 地点で停止する。既定停止時間は5秒 |
| `wait_time` | 省略可 | 停止時間[秒]。`stop`より優先 |

通常区間は`NavigateThroughPoses`の１つのactionで走行する。距離半径で手前の点を消す`RemovePassedGoals`は使わない。専用の`RemoveTraversedGoals`が再計画周期（約3秒）とは独立してBTの毎周期で動作し、その残り点数フィードバックから現在の未通過番号を更新する。中間点でgoalを再送しないため、各点を短距離の最終ゴールとみなすことによる減速を減らせる。

through_posesでは`default_threshold`とYAMLの`threshold`を中間点の削除に使わない。進行方向で地点を越えた場合、または角を曲がって次の区間側へ進んだ場合に通過済みとして除外する。1 m以内に近づいただけでは除外しない。通過後の古い地点を再計画で再び目指さないため、通過済み管理そのものは残す。これは点への厳密な位置・姿勢合わせを保証するものではなく、それが必要な点は停止点とする。元のYAMLを変更する必要はない。

最終点はNav2が姿勢を含めて`SUCCEEDED`を返すまで待つ。`stop`、`wait_time`、有効な`person_area`を持つ点で区間を終え、それらの到着には従来の`precise_threshold`（既定0.35 m）とキャンセル確認を使う。人物探索のACK・完了通知・タイムアウトと、Navigation-onlyで探索点を通常点として扱う機能は変更しない。

2026-10-22から、through_posesで一度に送る区間は`max_section_length`（既定40.0 m）でも分割する。現在位置から各waypointを順につなぐ折れ線の長さを合計し、40 mを超える前の点を区間終端とする。実機・sim共通で、to_poseには適用しない。停止・待機・人物探索や急な曲がりによる分割が先に来る場合は、短い区間を使う。距離上限だけで分割した通常区間では、終点まで`section_switch_distance`（既定6.0 m）以内になると次の区間へ更新する。古い区間の未通過点も新しい目標列の先頭に残し、通過判定は従来の幾何判定を使う。停止・待機・人物探索・最終点・急な折り返し・終端近接による分割では早期更新せず、到着確認を維持する。`section_switch_distance:=0.0`をROS引数で指定すると通常区間も到着完了を待つ。障害物を避ける実際の計画経路長の厳密な上限ではない。また、次の1点だけで40 mを超える場合は、その点を単独で送り、元の地点やタスクを飛ばさない。

通常点でも、90度以上の急な方向転換・切り返しでは、その点を区間終端として分割する。また、区間終端が経路の途中部分のすぐ近くにある場合も分割する。MPPIが未走行の長い経路を残したまま「ゴール付近」と評価することや、折り返しの往路・復路を同時に追うことを避けるためである。分割点は飛ばさず、Nav2の到着確認後に次の区間を送る。直線区間はまとめて走り、急な切り返しの区間境界では減速・停止と向き合わせが入る。これは区間の選び方の変更であり、中間点を距離で早期削除する判定は戻していない。

区間終端が現在位置とほぼ同じ場合は、未走行の経路を到達済みと誤認しないよう、その手前の遠い点で区間を分ける。閉じた経路やループの折り返しでは、この分割点で減速・停止する場合がある。曲がり角・障害物・本来の停止点での減速も引き続き行う。

ランチャーの実機・sim両方の追従コマンドで、起動時に走行方式を選択できる。`1`は`navigation_mode:=to_pose`、`2`は`navigation_mode:=through_poses`を渡す。方式を変更する場合はwaypoint追従を停止して起動し直し、選び直す。通過管理プラグインを初めて導入する際は`robotbase_bt_nodes`もビルドし、Nav2も再起動する。

サンプルは`maps_waypoints/waypoints/koko-sim-example.yaml`にある。

## Nav2標準Waypoint Follower

`robotbase_bringup/launch/nav2.launch.py`はNav2標準の`waypoint_follower`もLifecycle管理下で起動する。実機・シミュレーションの設定はそれぞれ`params/real/nav2.yaml`と`params/sim/nav2.yaml`内の`waypoint_follower`に置く。RViz Navigation 2パネルなど、`/follow_waypoints` actionを使うクライアント向けである。

`koko_waypoint_follow*`はYAMLの距離閾値やループに対応する独自クライアントで、標準サーバーの`/follow_waypoints`ではなく`/navigate_through_poses`を使う（旧方式を明示した場合は`/navigate_to_pose`）。同時に別クライアントからナビゲーションgoalを送らない。

## 停止と異常終了

待機地点または追従失敗では、次の順で停止する。

```text
ゼロTwist -> /cmd_vel_direct -> twist_mux -> /cmd_vel
                                       ↓
                             /stop=trueで入力をロック
```

Gazeboや一部モータdriverは最後に受けた速度を保持するため、`/stop`だけでmux出力を無通信にすると停止しない可能性がある。このためゼロ指令を先に通す。追従端末を`Ctrl+C`で閉じると、現在goalをキャンセルしてゼロ指令を送り、スクリプトの終了処理が`/stop=false`を送る。

## 走行中の手動一時停止・再開

同じROS Domainの端末から呼び出す。simなら先に`koko_sim_src`、実機なら`koko_src`で環境を合わせる。

```bash
# 一時停止
ros2 service call /koko_waypoint_follower/set_paused std_srvs/srv/SetBool '{data: true}'

# 再開
ros2 service call /koko_waypoint_follower/set_paused std_srvs/srv/SetBool '{data: false}'
```

一時停止ではゼロ速度を送り、現在のactionをキャンセルし、通過済み番号を保持する。Nav2の終了確認と停止指令の整定後に再開すると、未通過の点から残りの区間を送り直す。キャンセル未確認の間に再開を要求した場合は`success: false`を返すので、応答のメッセージに従って再度呼び出す。確認がタイムアウトした場合は停止して終了する。

この操作は移動中の一時停止用。指定点での停止移行・時間待機・人物探索中は、既存の処理を中断したり探索を飛ばしたりしないため、一時停止要求を受け付けない。終了したノードを再起動するときは、従来どおり開始番号を指定する。

## 2026-10-20の検証

40 × 30 mの屋内施設simで2 m間隔の直線経路を検証した。開始番号2、走行中の一時停止・再開、指定点で1秒待機、模擬Cameraの到着ACK／完了通知、その後の最終点への到着を確認した。通常通過点x=2 mとx=6 mでのodom前進速度はそれぞれ平均約0.899、0.898 m/sだった。これはsimでの確認であり、実機の走行品質は別途確認が必要。

区間分割の修正後は、実際の`sim_test.yaml`のコピーを専用Domainで使用し、WP1〜18（細かな切り返し）とWP34〜60（往復・停止点）をそれぞれ完走した。検証は`localization:=static`で行い、一時停止・再開とWP46の10秒待機も確認した。両方の実行で`Failed to make progress`と失敗点のスキップは0件。通常の直線通過時は約0.9 m/sを維持した。実行中の利用者のDomainや元のYAMLは変更していない。実機とAMCLを含む同条件での再確認は別途必要。

## 切り分け

```bash
ros2 action list | grep -E 'navigate_through_poses|navigate_to_pose|follow_waypoints'
ros2 lifecycle get /waypoint_follower
ros2 run tf2_ros tf2_echo map robot/base_footprint
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 topic echo /stop
```

- `Waiting for /navigate_through_poses`: Nav2未起動、またはDomain違い
- TF unavailable: AMCL/SLAM未起動、初期姿勢未設定、TF接頭辞違い
- goal rejected/aborted: Nav2端末のplanner/controller/costmapエラーを確認
- goalは進むが車体が動かない: twist_mux、`/stop`、`/cmd_vel`以降を確認

## ビルドとテスト

```bash
cd ~/robotbase_ws
colcon build --symlink-install --packages-select robotbase_waypoint robotbase_bringup
source install/setup.bash
colcon test --packages-select robotbase_waypoint
colcon test-result --verbose
```

### 40 m区間・4 m手前更新の検証（2026-10-22）

中之島WP30〜55の約106 mを、独立Domain 212・static自己位置推定で走行。更新後は128.78 sim秒で完走し、空経路エラー・中断・失敗点スキップ0回、早期更新2回。従来の40 m区間・到着待ちの140.52秒に対し約11.7秒短い（各1回で負荷変動を含む）。距離3〜100 mの区間は静止0秒、最低速度約0.324 m/sで、減速は残る。全125地点・AMCL・実機での確認は別途必要。停止・待機・人物探索・折り返し保護、旧goalの遅延結果、更新中の一時停止と両goalのキャンセルなどを含め、packageテスト67件成功。

### 更新後の成功通知の確認

sim_nkano4では4 m切替時に旧区間の到着で更新後のactionが成功し、離れた新終点まで到達済みと扱う事象が2回確認された。早期更新したactionの成功時はTFで新終点までの距離を確認し、precise_threshold（既定0.35 m）より遠い、またはTFが取得できない場合は進捗を終点へ飛ばさず未通過点を再送する。通常区間の切替距離の既定値は6 mへ変更。切替後の待ち時間は再計画周期と経路計算の両方を含む。
