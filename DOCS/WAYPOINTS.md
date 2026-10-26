# ウェイポイント記録・追従

## 概要

SIRIUSの`move_goal.py`と位置記録ツールを参考に、ココちゃん専用の`robotbase_waypoint`パッケージを追加した。実機・シミュレーションで同じYAMLを使い、起動時にNav2の`NavigateToPose`（従来方式・既定）と`NavigateThroughPoses`（連続走行）を選択できる。

SIRIUSから引き継いだもの:

- `format_version: '1.0'`のウェイポイントYAML
- 開始番号、ループ、通常の中間点切替距離
- 地点ごとの`threshold`、`stop`、`wait_time`
- 現在位置の1点保存と、一定移動距離ごとの連続記録

ココちゃん向けに変更したもの:

- 固定の`sirius3/base_footprint`を廃止し、`robot.env`のTF接頭辞を使用
- SIRIUSの固定パスを廃止し、`robotbase_ws/maps_waypoints/waypoints/`へ集約
- LLM用`/nav_control`、地図切替shell、SIRIUS固有トピックを含めない
- 従来方式の通常中間点では次のgoalを送り、Nav2側でpreemptする。停止・待機・人物探索では既存の停止処理を使う
- 停止時は`/cmd_vel_direct`へゼロ速度を送り安全に停止を維持する
- 失敗時は`stop_on_failure: false`（既定）でスキップして次のウェイポイントへ自動遷移する
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

追従コマンドはYAML、開始番号、走行方式、切替距離、ループ有無を質問する。方式1では中間点の切替距離（既定1.0 m）、方式2では通常区間の切替距離（既定6.0 m）を入力する。

```text
走行方式を選択：
  1: NavigateToPose（従来方式）
  2: NavigateThroughPoses（連続走行）
番号 [1]:
```

空入力は1。両コマンドとも`navigation_mode:=to_pose`または`navigation_mode:=through_poses`を明示的に渡し、選択を端末とノードの起動ログに表示する。ノードを直接起動して`navigation_mode`を省略した場合も`to_pose`になる。

- `to_pose`: 1点ずつ送信する。通常中間点の切替距離、各点の`threshold`、最終点のNav2成功待ち、停止・待機・人物探索の判定は従来どおり。
- `through_poses`: 通常の中間点をまとめて送り、停止・待機・人物探索の点を区間終端にする。急な切り返し、区間終点付近を先に通る経路、現在地に終点が重なるループも区間分割し、YAML順で送る。通常中間点の削除は`RemoveTraversedGoals`の通過判定で行い、`default_threshold`やYAMLの`threshold`を早期削除に使わない。区間終端の停止・人物探索では既存の精密到着・停止処理を使う。

`through_poses`は`max_section_length`（既定40.0 m）でも区間を分ける。現在位置から未通過waypointを順につないだ折れ線距離を合計し、上限を超える前の地点を終端にする。固定点数や始点・終点間の直線距離は使わない。最初の1点への距離だけで上限を超える場合は、その点を単独で送り、地点を飛ばさない。障害物回避を含む計画経路の長さを制限するものではない。

距離上限だけで分割した通常区間は、終端まで`section_switch_distance`（既定6.0 m）以内で次の区間へ更新する。旧区間の未通過点も新しい目標列に残し、新終端が先へ延びる場合だけ更新する。通過済み番号はBTの`RemoveTraversedGoals`が返す残り点数で管理し、距離だけでは進めない。停止・待機・人物探索・最終地点、急な折り返しや終端近接による分割では早期更新せず、従来の到着確認を維持する。直接起動時に`section_switch_distance:=0.0`を渡すと通常区間も到着完了を待つ。

早期更新後は、旧区間のFollowPath完了が新しいaction全体の成功として通知される場合があるため、成功通知に加えてTFで新終端への到達を確認する。`precise_threshold`（既定0.35 m）より遠い、または位置が取得できない場合はindexを終端へ進めず、未通過地点を保持して再送する。旧goalの遅延した結果・feedbackは新しい進捗に反映しない。区間失敗時は次の未通過点を単独で再試行し、終了・一時停止時は更新前後の未終了goalをすべて管理・キャンセルする。

方式変更は追従を`Ctrl+C`で終了し、同じ追従コマンドを再起動して選択し直す。通常の切替でNav2全体の再起動は不要。今回のプラグインを導入する初回だけは、ビルド後にNav2を起動し直してライブラリを読み込む。

走行中の一時停止・再開は追従ノードのサービスでも行える（同じ実機／sim環境の端末を使う）。

```bash
ros2 service call /koko_waypoint_follower/set_paused std_srvs/srv/SetBool '{data: true}'
ros2 service call /koko_waypoint_follower/set_paused std_srvs/srv/SetBool '{data: false}'
```

停止要求ではgoalのキャンセルとゼロ速度を維持し、キャンセル終了と停止安定を確認してから再開する。再開応答が待機中を示した場合は、確認後に再試行する。停止・待機・人物探索処理中の要求はそのタスクを変更しない。人物探索の既存トピック、ACK、完了通知、タイムアウト、Navigation-onlyの扱いは共通。

選択するYAMLと、Nav2で選択した地図の座標系が一致している必要がある。

MPPIは後退より前進を強く優先する。最初の地点は、現在姿勢から見て前方に置く。後方から開始すると、到達まで大きく旋回する場合がある。

## RVizでウェイポイントを表示

追従ノードは、読み込んだ全地点の番号・向きの矢印・地点を順に結ぶ線を
`/waypoint_markers`（`visualization_msgs/msg/MarkerArray`）へ配信する。
RViz設定には「Waypoints」を追加済み。停止・待機地点の矢印は黄色、通常地点は緑色になる。

走行前に表示だけ確認する場合は、追従ノードを起動せず次を実行する。
このコマンドはナビゲーションgoalや速度指令を送らない。

```bash
source install/setup.bash
ros2 run robotbase_waypoint waypoint_visualize --ros-args \
  -p waypoint_file:="$PWD/maps_waypoints/waypoints/1024-nakanoshima-2.yaml"
```

RVizのFixed Frameを`map`にする。すでに開いているRVizでは、Add → MarkerArrayを選び、
Topicを`/waypoint_markers`に設定する。後から開いたRVizにも表示される。
YAML変更後は表示ノードを再起動する。追従を開始する前に表示専用ノードを`Ctrl+C`で終了し、
同じトピックへ複数のルートを同時配信しないようにする。

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
| `threshold` | 省略可 | `to_pose`でこの中間点から次へ切り替える距離[m] |
| `stop` | 省略可 | 地点で停止する。既定停止時間は5秒 |
| `wait_time` | 省略可 | 停止時間[秒]。`stop`より優先 |

`to_pose`では通常の中間点は距離閾値で次の`NavigateToPose` goalを送り、現在のgoalをNav2内で更新する。この切替では明示的なキャンセルやゼロ速度指令を送らない。最終点はNav2が姿勢を含めて`SUCCEEDED`を返すまで待つ。`stop`または`wait_time`を持つ中間点は従来どおり停止し、`precise_threshold`（既定0.35 m）を使用する。

サンプルは`maps_waypoints/waypoints/koko-sim-example.yaml`にある。

## Nav2標準Waypoint Follower

`robotbase_bringup/launch/nav2.launch.py`はNav2標準の`waypoint_follower`もLifecycle管理下で起動する。実機・シミュレーションの設定はそれぞれ`params/real/nav2.yaml`と`params/sim/nav2.yaml`内の`waypoint_follower`に置く。RViz Navigation 2パネルなど、`/follow_waypoints` actionを使うクライアント向けである。

`koko_waypoint_follow*`はYAMLの距離閾値やループに対応する独自クライアントで、標準サーバーの`/follow_waypoints`ではなく、選択した方式に応じて`/navigate_to_pose`または`/navigate_through_poses`を使う。同時に別クライアントからナビゲーションgoalを送らない。

## 停止と異常終了

待機地点または追従失敗では、次の順で停止する。

```text
ゼロTwist -> /cmd_vel_direct -> twist_mux -> /cmd_vel
                                       ↓
                             /stop=trueで入力をロック
```

Gazeboや一部モータdriverは最後に受けた速度を保持するため、`/stop`だけでmux出力を無通信にすると停止しない可能性がある。このためゼロ指令を先に通す。追従端末を`Ctrl+C`で閉じると、所有するgoalをキャンセルしてゼロ指令を送る。終了処理はgoal終了の確認とゼロ速度の安定時間を満たした場合に`/stop=false`を送り、確認できない場合は停止要求を維持する。

## 切り分け

```bash
ros2 action list | grep -E 'navigate_to_pose|follow_waypoints'
ros2 lifecycle get /waypoint_follower
ros2 run tf2_ros tf2_echo map robot/base_footprint
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
ros2 topic echo /stop
```

- `Waiting for /navigate_to_pose`: Nav2未起動、またはDomain違い
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
