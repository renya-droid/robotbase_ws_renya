# 引き継ぎ：レストラン人物シミュレータと配膳ロボット予備実験（2026-10-31）

別のPCの Claude Code へ：まずこのファイルと、`AGENTS.md`、`src/restaurant_nav_experiment/README.md` を読んでください。会話履歴は引き継がれていません。

## 研究の問い
外部カメラで得た「人の位置」と「着席しているか歩いているか」を Nav2 の Costmap に入れると、配膳ロボットの走行は改善するか。今の段階では位置にシミュレータの正解値を使い、効果の上限を測っている。

## 報告書（claude.ai の文書、同じアカウントで開ける）
- シミュレータの調査報告：https://claude.ai/artifact/3Gkzag5MqQjxxoMtnbBmQW
- 配膳ロボット予備実験報告：https://claude.ai/artifact/E4me6aBswgq8g39yrwmJhQ
- 同じ内容の下書き：`artifacts/restaurant_nav/REPORT.md`

## 今の状態
- `src/restaurant_sim`：人の動き（入店・着席・給水・会計・退店）。Ralph worktree（`.runtime/restaurant-issues-20261030`）の改良を取り込み、さらに次を追加した。
  - 会計・給水のFIFO列、固定刻みの時計、`failure_policy`（fail/log）、曲がった通路での退避、遠回りの上限、列が満員なら着席のまま待つ
  - seedの指定、ロボットの近くに人を出さない、ロボットを3点として扱う、`pose_transport:=ros`
  - 擬似時計では low/medium/high × seed 11/22/33/44/55 の15通りすべてで全員が退店した。テスト38件合格。
- `src/restaurant_nav_experiment`（新規）：ロボット＋人のGazebo、条件B0/B1/P、配膳の自動実行と記録、集計。テスト8件合格。
- 予備実験の結果（high、6 seed）：**条件間に統計的にはっきりした差はない**（差の95%区間はすべて0をまたぐ）。B1→Pで成功20→22/29、所要時間は平均−3.6秒（95%区間 −17〜+10秒）。詳細は報告書。

## 決まっていること（ユーザーの判断）
- 配膳は「キッチンカウンター東側の基地 → 着席客のいる席 → 基地」。
- 予備実験では、人はロボットに近づかれたら止まるだけ（よけない）。後でパラメータ化する。
- 位置は正解値でよい（まず効果の上限を見る）。
- コミットと push はユーザーが自分で行う。AGENTS.md で自動の commit/push は禁止。

## このPCでつまずいた点（別のPCでも確認すること）
- **ビルド**：ワークスペースに navigation2 のソースと実機ドライバがあるので、そのまま `--packages-up-to` するとそれらまで巻き込む。Nav2 は `/opt/ros/jazzy` のバイナリ（1.3.11）を使い、必要な7つだけをビルドした：
  ```bash
  source /opt/ros/jazzy/setup.bash
  KEEP="velodyne_laserscan robotbase_bt_nodes robotbase_bringup robotbase_sim external_camera_people_layer restaurant_sim restaurant_nav_experiment"
  IGN=$(colcon list -n | grep -v -x -F -f <(echo $KEEP | tr ' ' '\n') | tr '\n' ' ')
  colcon build --packages-select $KEEP --packages-ignore $IGN --cmake-args -DBUILD_TESTING=OFF
  ```
- **twist_mux がこのPCに入っていない**ので、実験用の `robot_sim.launch.py`（`robotbase_sim/sim.launch.py` のコピー）では `twist_mux_lite` を使っている。別のPCに twist_mux があれば元に戻せる。
- 長時間の実行中に**ノートPCが自動でスリープ**して1回分が無効になった。`systemd-inhibit` で実行中だけ抑止した。
- 実験は専用の ROS ドメイン94・Gazebo partition `restaurant_nav_exp` で動かす。robot.env は読み込まない。
- `rclpy` を使うテストは `source /opt/ros/jazzy/setup.bash && source install/setup.bash` の後に実行する。`PYTHONPATH=.` を付けると ROS のパスが消える。

## 研究上の注意（結果を読むとき）
- 18席中6席（中央の菱形テーブル）は、ロボット（0.90×0.66 m）が配膳位置に入れない。
- 元のロボット設定では、LiDAR（高さ0.72 m・下向き15°）から高さ0.45 mの椅子が約1 m以内で見えず、Local Costmapに地図もない。実験では全条件で Local Costmap に静的地図を追加した。実機でも起こりうる。
- 人がロボットのそばで動けなくなる停止が、全条件で1回あたり0〜16件記録された（人は止まるだけでよけないため）。
- 基地 world (7.45, 3.1)、配膳位置の選び方、席の前で待つ5秒は仮定。
- 負荷が高く、Gazebo の実時間比は約0.7だった。

## 次にやること（提案、優先順）
1. 人がロボットをよける・譲る反応を入れ、譲る確率をパラメータにする。
2. 人の円を Local Costmap にも入れる条件を追加する。
3. seed で座席・到着・行動まで変わるようにし、試行数を増やす。
4. 店の混雑を一定に保つシナリオを作る。
5. カメラの誤差・遅延・死角のモデルを入れる。
6. 実機の Local Costmap が低い椅子を捉えられるか確かめる。

## 再現
```bash
cd ~/robotbase_ws_renya
source /opt/ros/jazzy/setup.bash && source install/setup.bash
src/restaurant_nav_experiment/scripts/run_condition.sh P high 42 artifacts/restaurant_nav/try_P   # 1条件（約7〜9分）
src/restaurant_nav_experiment/scripts/run_pilot.sh artifacts/restaurant_nav/pilot_high "B0 B1 P" "42 11 22 33 44 55" high start_sim:=40.0 stop_sim:=330.0
python3 src/restaurant_nav_experiment/scripts/paired_stats.py artifacts/restaurant_nav/pilot_high
```

## git に入っていないもの
- `.runtime/`（追跡対象外）：ランチャー2の実体 `.runtime/restaurant-launcher`（ブランチ `launcher/restaurant-desktop`）と、Ralph worktree。ランチャー2の変更はそのブランチで別にコミットする。
- 実験の生ログ（`launch.log`・`ros_log/`）。結果のCSV・JSON・集計は入れてある。
