# Nav2でパスは生成されるが実機が動かない

## 1. 事象

基準日: 2026-08-19

実機で確認できたこと:

- 通常の手動操作で車輪が動く
- `keyop2` で車輪が動く
- Nav2へゴールを与えるとRViz上でパスが生成される
- Nav2では実機が動き始めない

まだ観測していないこと:

- `/cmd_vel_nav` に非ゼロ指令が出たか
- `/cmd_vel_smoothed` に指令が届いたか
- `/cmd_vel` にNav2由来の指令が届いたか
- Nav2端末の完全なエラーログ
- goal実行時のcontroller lifecycleとlocal costmap状態

その後、専用設定へ置換したNav2で実機自律移動とSLAMしながらの走行には成功した。ただし上記4項目を当時の条件で記録していないため、最初の無走行の直接原因は確定していない。以下は旧設定から確認できた最有力原因と、再発時の診断記録として残す。

## 2. 結論

### 旧最有力: RT優先度権限がないのに有効化されていた

旧 `params/nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    use_realtime_priority: true
```

Nav2の `ControllerServer` はこの値をFollowPathの `SimpleActionServer` へ渡す。ゴールを受けた非同期スレッドは次の順に実行する。

```text
setSoftRealTimePriority()
work()  # 実際のFollowPath制御ループ
```

`setSoftRealTimePriority()` は `SCHED_FIFO`、priority 49を要求し、失敗時に例外を投げる。例外は `work()` より前なので、権限がなければパス計画後の速度計算へ進めない。

確認した現在のシェル環境:

```text
$ ulimit -r
0
```

上限0では通常ユーザーがpriority 49を設定できない。同じログイン環境で実機Nav2を起動したなら、次の症状を一貫して説明できる。

1. plannerは動くのでパスが描画される
2. FollowPath goalは受け取る
3. controllerの実行スレッドがRT設定で終了する
4. `/cmd_vel_nav` が出ず、ロボットは動こうとしない

例外が非同期future内に保持され、端末へ明瞭に表示されない可能性もある。ログにメッセージがないことだけでは除外できない。

### 適用済みの対策

新しい実機用 `params/real/nav2.yaml` は次の値で固定した。

```yaml
use_realtime_priority: false
```

動的な `ros2 param set` だけでは、既に生成済みのaction serverへ反映されない可能性がある。YAML変更後に `controller_server` を含むNav2全体を再起動する。

実機再試験で `/cmd_vel_nav` が出て走行開始すれば、旧RT設定が原因だったと判断できる。RTを将来有効化する場合だけ、次を確認する。

RTを許可する場合は `/etc/security/limits.conf` 等へユーザーの `rtprio` 上限を設定する。設定後、Nav2を起動する同じターミナルで次を確認する。

```bash
ulimit -r
chrt --fifo 49 true
```

`ulimit -r` が49以上で、`chrt` がエラーなく終了する必要がある。

## 3. 「トピックが違うか」の回答

主速度経路の名前はコード上は一致している。

```text
controller_server:  /cmd_vel_nav をpublish
velocity_smoother:  /cmd_vel_nav をsubscribe
velocity_smoother:  /cmd_vel_smoothed をpublish
twist_mux:          /cmd_vel_smoothed をsubscribe
twist_mux:          /cmd_vel をpublish
Roboteq:            /cmd_vel をsubscribe
```

従って、設定ファイル同士の単純な名前違いが第一原因ではない。

ただし次の運用条件がある。

- `twist_mux` は `nav2_real` と別起動
- `keyop2` はmux経由
- 通常手動操作は `/cmd_vel` へ直接出せる
- 別ワークスペースを後からsourceすると、異なるlaunch/configを使う可能性がある

旧設定にあった未接続のcollision monitorは、新しい最小launchとparamsから削除した。従って今回の無走行原因にはならないが、独立した最終停止レイヤーがない点は別の安全課題として残る。

## 4. 実機での最短切り分け

### 4.1 試験前

Nav2を起動するターミナルでoverlayを確認する。

```bash
source ~/robotbase_ws/install/setup.bash
ros2 pkg prefix robotbase_bringup
ros2 pkg prefix roboteq_ros2_driver
ulimit -r
```

prefixはすべて `~/robotbase_ws/install` 配下を指すこと。新設定の `use_realtime_priority` は `false`。

### 4.2 必須ノードとLifecycle

```bash
ros2 node list | sort
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /bt_navigator
ros2 node info /twist_mux
ros2 node info /roboteq_ros2_driver
```

期待値:

- Nav2 lifecycle nodeは `active`
- `/twist_mux` が存在
- Roboteqが `/cmd_vel` をsubscribe

### 4.3 ゴール中の速度境界

ゴール送信中に各コマンドを別ターミナルで実行する。

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_smoothed -v
ros2 topic info /cmd_vel -v

ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
```

必要なら実値も表示する。

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

判定表:

| 最初に止まる場所 | 判断 | 次に見るもの |
|---|---|---|
| `/cmd_vel_nav` が無い | controllerが速度計算を開始できていない | RT権限、controllerログ、odom、local costmap |
| navは出るがsmoothedが無い | velocity smootherが非active、型/QoS/input不一致 | lifecycle、`node info` |
| smoothedは出るが`/cmd_vel`が無い | mux未起動、lock、別設定を読んでいる | mux node、`/stop`、package prefix |
| `/cmd_vel`が非ゼロだが動かない | driver以降 | Roboteq subscriber、非常停止、motor command |
| 全段がゼロ | controllerが有効軌道を選べていない | local costmap、scan、TF、MPPIログ |

手動走行と`keyop2`が既に成功しているため、`/cmd_vel`以降の故障可能性は低い。特に`keyop2`は `/cmd_vel_teleop -> twist_mux -> /cmd_vel` を通るので、同じ試験時に成功したならmuxとRoboteqの後半は正常と推定できる。

## 5. 次点の原因候補

### 5.1 `/odom/filtered` またはTFがない

Nav2のcontroller、velocity smoother、BTは `/odom/filtered` を参照する。local costmapは `robot/odom` をglobal frameとして使う。

```bash
ros2 topic hz /odom
ros2 topic hz /odom/filtered
ros2 topic echo /odom/filtered --once
ros2 run tf2_ros tf2_echo robot/odom robot/base_footprint
ros2 run tf2_ros tf2_echo map robot/base_footprint
```

主な失敗条件:

- `sf_real` 未起動
- `/imu` または `/odom` のtimestamp/値が不正
- Roboteqを `pub_odom_tf:=false` で起動したのにEKFがTFを出していない
- 別ノードが同じTFを二重配信
- frame名が `base_footprint` と `robot/base_footprint` で分裂

グローバルパスが描けても、local controllerに必要な最新TFが取れず速度を出せない場合がある。

### 5.2 `/scan3`、`/hokuyo_scan` またはsensor TFがない

実機用 `params/real/nav2.yaml` はAMCLで `/scan3` を参照し、global/local costmapで `/scan3` と `/hokuyo_scan` の両方を参照する。

```bash
ros2 topic hz /scan3
ros2 topic hz /hokuyo_scan
ros2 topic info /scan3 -v
ros2 topic info /hokuyo_scan -v
ros2 topic echo /scan3 --once
ros2 topic echo /hokuyo_scan --once
ros2 run tf2_ros tf2_echo robot/base_footprint <scan3_header_frame_id>
ros2 run tf2_ros tf2_echo robot/base_footprint robot/hokuyo_link
```

見る点:

- `sensor_msgs/msg/LaserScan` か
- `header.frame_id` からbaseまでTFがあるか
- timestampが現在時刻か
- 距離値が全て0、NaN、範囲外でないか

VLP-16 driverと改造版 `velodyne_laserscan` が複数リング合成の `/scan3` を生成し、Ethernet Hokuyoが足元180°の `/hokuyo_scan` を生成する。単一リング `/scan` はNav2入力に使用しない。

### 5.3 local costmapが未知または障害物で埋まる

現在のlocal costmap:

- rolling 6 x 6 m
- 0.05 m/cell、free space初期化
- `/scan3`と`/hokuyo_scan`用の独立したobstacle layer
- 各sourceの`max_obstacle_height: 2.0` m
- inflation radius 0.70 m
- footprint 0.90 x 0.66 m

`scan.max_obstacle_height` は観測source内に必要で、未指定時の既定値は0.0 m。VLP-16の取付高さが0 mより上の場合、これがないとスキャン点が全件除外され、配信トピックが存在してもlocal costmapは全セル0になる。反対に、センサー自己点、誤TF、地面反射でロボット周囲がlethalになるとMPPIは有効軌道を出せない。

RVizで次を同時表示する。

- Local Costmap
- Local Footprint
- LaserScan `/scan3`、`/hokuyo_scan`
- MPPI optimal trajectoryとcandidate trajectories（必要時）
- RobotModelとTF

Nav2端末では次を探す。

```text
No valid trajectories
No valid control
Costmap timed out
Sensor origin ... out of map bounds
Message Filter dropping message
Timed out waiting for transform
```

### 5.4 muxの高優先度入力またはstop lock

`twist_mux` の優先度は次の順。

```text
/stop             255
/cmd_vel_teleop   100
/cmd_vel_direct    90
/cmd_vel_smoothed  10
```

高優先度publisherがゼロ速度を連続送信してもNav2は選ばれない。

```bash
ros2 topic info /cmd_vel_teleop -v
ros2 topic info /cmd_vel_direct -v
ros2 topic info /stop -v
```

`keyop2` は非ゼロ速度を保持している間、100 ms周期でpublishを続ける。`s` で速度を0へ戻し、約1秒待ってからNav2を試す。緊急停止を使った場合は `e` で解除する。

### 5.5 twist_mux起動漏れ

```bash
ros2 node list | grep '^/twist_mux$'
ros2 topic info /cmd_vel_smoothed -v
```

`/cmd_vel_smoothed` にsubscriberがなければmuxがない。`twist_mux` エイリアスを起動する。

今回`keyop2`が同じ試験構成で動いたなら、mux起動漏れの可能性は低い。ただし通常手動操作だけが動いた場合はmux未起動でも説明できる。

### 5.6 stale build / 別overlay

ソースを直してもinstall側が古い、または `sirius_jazzy_ws` を後からsourceしていると、見ているコードと実行コードが異なる。

```bash
ros2 pkg prefix robotbase_bringup
ros2 pkg prefix roboteq_ros2_driver
ros2 pkg executables robotbase_keyop
```

必要なら再ビルド後、新しいターミナルでrobotbaseだけをsourceする。

## 6. 推奨する再試験手順

1. robotbaseを再ビルド・sourceする（新設定は `use_realtime_priority: false`）
2. `koko_roboteq`, `koko_velodyne`, `koko_imu`, `koko_sf_real`, `koko_twist_mux`, `koko_nav2_real` の順に起動する
3. `/odom/filtered`、`/scan3`、TFを確認する
4. `keyop2` は終了するか、`s` 後1秒待つ。`/stop` を解除する
5. Nav2ゴールを送る
6. `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel` を同時記録する
7. Nav2端末ログを保存する

最小rosbag例:

```bash
ros2 bag record \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel \
  /odom /odom/filtered /scan3 \
  /tf /tf_static \
  /plan /local_plan \
  /local_costmap/costmap /global_costmap/costmap
```

環境によって存在しないtopicは事前に `ros2 topic list` で調整する。

## 7. 解決後に記録する項目

再試験後、この節へ次を追記する。

- 確定原因
- 再現条件
- 最初に途切れていたtopic
- Nav2ログの代表メッセージ
- 採用した修正
- 修正コミット
- 実機で成功した起動順
- 直進、旋回、停止、手動割込みの試験結果

現時点の判断は「旧RT権限不一致が最有力で対策済み、主速度topic名は整合、実機再試験待ち」である。
