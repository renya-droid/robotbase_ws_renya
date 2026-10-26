# Waypoint走行方式の移植・検証（2026-10-20）

## 対象と変更範囲

main `c72dc89` に対し、ユーザー指定の `origin/renya/camera-nav` (`0ed7a38`) からwaypoint関連だけを移植。開始時のmain作業ツリーは変更なし。ブランチ切替、commit、pushは実施していない。

- 実機・sim共通の起動スクリプトに走行方式の選択を追加。既定1、不正入力を拒否。`to_pose`だけ距離を質問し、整数入力もROSのDOUBLEとして渡す。`through_poses`では`default_threshold`を渡さない。
- ノードの既定を`to_pose`に変更。元mainの1点送信・各threshold・最終点の成功待ち・停止・人物探索を維持。区間分割・通過フィードバックは`through_poses`だけに適用。
- `through_poses`の通常中間点一括送信、タスク境界、急な切り返し・重複経路・現在地付近の終点に対する分割、途中開始、ループ、一時停止・再開を移植。
- `robotbase_bt_nodes`のC++ライブラリ、BT登録、CMake、manifestと、距離による早期削除を使わない`RemoveTraversedGoals`のBT XMLを移植。BT XMLはbringupの既存CMakeのDIRECTORYインストール対象。
- 実機・simのNav2 YAMLには`plugin_lib_names`の1行だけ追加。元mainには既存の同パラメータはなかった。YAML構造を比較し、他の値（速度、加速度、MPPI、デッドバンドなど）が完全一致することを確認。
- bringup→BTパッケージ、waypoint→bringup（BT XMLの参照）、`ament_index_python`、`std_srvs`の依存を登録。
- mainのlaunchでは両navigatorとBT XMLのパス設定が既に有効なため変更不要。ワールド選択、カメラ処理、PeopleLayerは移植していない。

## ビルドと自動テスト

ROS 2 Jazzy、現在のworkspaceのinstallを使用。次の4パッケージのビルドに成功。

```bash
cd ~/robotbase_ws_renya
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select robotbase_bt_nodes robotbase_bringup robotbase_waypoint robotbase_sim
```

| 検証 | 結果 |
|---|---|
| real/sim × 既定・1・2、距離引数の出し分け、不正方式・不正距離 | 起動スクリプト15件成功 |
| waypointパッケージ（既存回帰、人物探索ACK・完了・タイムアウト・Navigation-only、through区間分割、停止、ループ、途中開始、キャンセル確認、古いfeedback排除、to_poseの単一点送信・既定値・最終到着、lintを含む） | 58件成功 |
| C++通過判定（直線で直前に削除しない、角、逆向き、同一点） | 3件成功 |
| Bash構文、git diff --check | 成功 |

```bash
# パッケージ内で実行し、lintの探索をworkspace全体へ広げない。
(cd src/robotbase_waypoint && python3 -m pytest -q test)
python3 -m pytest -q tests/test_waypoint_startup.py
colcon test --packages-select robotbase_bt_nodes --event-handlers console_direct+
bash -n bash/startup_bash/waypoint_follow.sh
git diff --check
```

## Gazebo/Nav2での走行確認

専用ROS Domain 196、localhost内で、mainの`test_arena`、`gui:=false`、`localization:=static`（Gazeboのodomを使う既存モード）を使用。実機driverは起動していない。速度・制御器設定はmainの値のまま。

| シナリオ | 確認内容 |
|---|---|
| to_pose_default | mode省略で従来方式。y=1,2,3へ1点ずつ送信。最初のYAML threshold=0.4、次のdefault=0.6で切替、最後はNav2成功を待って終了 |
| through_straight_pause | y=2,1,0を3点一括送信。YAML threshold=10でも早期削除なし。一時停止でキャンセル終了を確認して再開。停止中の移動は約0.003 m |
| through_hairpin_overlap | y=1,3,2,0の往復重複経路。折り返しで2区間に分割し、YAML順に完走 |
| through_tasks | y=1→2（1秒待機）→3（stop、試験用stop_duration=1秒）→4（人物探索）→4.5。タスクで分割し、到着通知→ACK→完了通知→再開を確認 |
| through_midstart_navigation_only | start_index=2。人物探索マーカーを通常点として連続送信し、Cameraへの到着通知なしで完走 |
| through_loop_cancel | 1周完了後に次周の区間を送信し、SIGINTを1回送って安全終了を確認 |

最初の3シナリオでは同じNav2プロセスを維持して`to_pose`→`through_poses`を切り替えた。通常の方式切替にNav2の再起動は不要。

人物探索は既存のROSトピックへACKと完了を返すテスト用ノードを使用。カメラ認識そのものと実機走行は未試験。ACK未受信・探索タイムアウト、unsafe完了などは自動回帰テストで確認。

結果と各シナリオのYAML・追従ログは[検証記録](verification/waypoint_modes_20261020/)に保存。

### 検証中に確認した制限

- Nav2内部の`follow_path`受付待ちタイムアウトによる中断を1回確認。再試行で停止・人物探索シナリオは完走。該当Nav2ログを`nav2_timeout.log`に保存。Nav2のタイムアウト値は変更していない。
- 起動直後の1 m先に最初の停止点を置いた試験では、制御器の前進指令約0.072 m/sが既存の0.10 m/sデッドバンドで抑えられ、進まないケースを確認。最初の区間を2 mにした試験で停止・待機・探索を検証した。デッドバンドやMPPIを変更して解消したものではない。
- ソフトウェア描画を強制した初回GazeboはMesa内でクラッシュ。描画の環境変数を外した通常のheadless起動では走行できた。

## 実機での起動

このPCではビルド済み。新しい端末で環境とaliasを読み込み、初回導入時はNav2を再起動して新しいBTライブラリを読み込む。

```bash
cd ~/robotbase_ws_renya
source bash/bash_alias1.sh
source bash/bash_alias2.sh
# 通常のセンサー・自己位置推定・twist_muxを起動
koko_nav2_real
# RVizで初期姿勢を確認した後、別端末で
koko_waypoint_follow
```

YAML→開始番号→走行方式を選択。1または空入力なら従来方式で、切替距離も入力。2なら連続方式で、距離入力はない。続いてループ有無を選ぶ。Navigation-onlyは`koko_src`後に`bash bash/startup_bash/waypoint_follow.sh real --navigation-only`で起動できる。

方式を変えるときは追従を`Ctrl+C`で終了し、同じコマンドを再実行して選び直す。Nav2は起動したままでよい。sim側は`koko_waypoint_follow_sim`で同じ選択になる。一時停止・再開サービスなどは[WAYPOINTS.md](WAYPOINTS.md)を参照。
