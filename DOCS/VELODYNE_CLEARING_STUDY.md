# Velodyne観測範囲・障害物コスト消去の可視化（シミュレーション専用）

屋外でVelodyne由来の障害物コストが残留する問題を、Gazeboで再現・可視化する環境。
実機設定（`params/real/`、`robot.env`、実機ドライバ）は変更していない。

## 起動

```bash
# RVizで可視化（シナリオ1周、約110秒のシミュレーション時刻）
ros2 launch robotbase_sim velodyne_clearing_study.launch.py

# シナリオを繰り返す / 障害物を動かさずリング範囲だけ見る
ros2 launch robotbase_sim velodyne_clearing_study.launch.py scenario_loop:=true
ros2 launch robotbase_sim velodyne_clearing_study.launch.py scenario:=false

# 画面なしで1周して終了（試験記録用）
ros2 launch robotbase_sim velodyne_clearing_study.launch.py gui:=false rviz:=false \
  shutdown_when_done:=true output_dir:=$HOME/velodyne_study_result
```

- `ROBOTBASE_PARAMS_DIR` は他のsim launchと同じ規則（未設定時 `~/robotbase_ws/params`）。
  このWSのparamsを使う場合は `export ROBOTBASE_PARAMS_DIR=~/robotbase_ws_renya/params`。
- 既存シミュレーションと同時に動かさないこと（同一 `ROS_DOMAIN_ID` だと `/clock`, `/scan3` が衝突する）。
  別Domainで動かす例: `ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch ...`
- `gui:=true` でGazebo GUIも開く。

## 構成

| 要素 | ファイル | 内容 |
|---|---|---|
| ワールド | `src/robotbase_sim/worlds/velodyne_clearing_study.sdf` | 壁なし200 m四方の平地。歩行者・自動車・低い箱・参照壁は観測範囲外に待機 |
| コストマップ | `params/sim/velodyne_clearing_study.yaml` | `/velodyne_study/costmap`。実機 `global_costmap.velodyne_obstacle_layer` と同じ値（obstacle 15 m / raytrace 20 m / `inf_is_valid: false`）。inflation・static・Hokuyoなし |
| 可視化 | `src/robotbase_sim/scripts/velodyne_coverage_viz.py` | `/scan3` の選択処理をPythonで再現し、点・リング・消去レイを分類表示 |
| シナリオ | `src/robotbase_sim/scripts/velodyne_clearing_scenario.py` | 障害物の設置・移動・撤去、真値表示、残留セル計測 |
| RViz | `src/robotbase_sim/rviz/velodyne_clearing_study.rviz` | 下表の表示を登録済み |

### RVizの表示

| 表示 | 意味 |
|---|---|
| リング範囲 | リングごとの円錐台（仰角どおりに傾いた面）。色＝採用範囲（センサー最小距離0.2 m〜採用上限）、灰＝除外（上限〜地面交点）。太線＝円錐台の内縁・外縁、細線＝30°ごとの母線、地面上の線＝地面との交点。ラベル＝仰角・上限・上限地点の高さ・地面距離。MarkerArrayのNamespacesで `R1 -13° 採用` などリング単位に表示を切替できる |
| 断面図 | ロボット左方向（`profile_azimuth_deg`、既定+90°）の縦断面。各リングのビームを、色＝採用区間、灰＝除外区間（地面まで）で描く。地面線に1 m刻みの目盛り |
| 消去レイ | 地面付近に描画。緑＝`/scan3`が有限（Nav2が消去する方向）、赤＝`+Inf`（`inf_is_valid:false`で消去されない方向） |
| 採用点 / 除外点 | 選択リングの点のうち採用上限内（緑）／上限超（橙）。全点群は既定で非表示 |
| scan3採用の3D点 | 各方向で実際に `/scan3` の値になった点（白）。登録された物体の高さを確認できる |
| 消去候補 clear_candidate | 方向ごとの選択リング生距離の最大値（参考、Nav2未使用）。既定で非表示 |
| 障害物の真値 | シナリオが指令した位置（半透明） |
| 残留コスト | 真値の外側にある致死セル（マゼンタ） |

`use_imu_tilt:=true` で地面交点を `/imu` のroll/pitchで傾けて描画する（シミュレーションの平地では変化しない）。

## 判定方法と前提（推定と確認済みの区別）

- 真値はシナリオが `/world/velodyne_clearing_study/set_pose` へ送った指令姿勢。ロボットは原点で静止させ、
  `odom` = ワールド座標として比較する。`/odom` から原点からのずれを記録（2026-11-09の試験では0.0 m）。
- 各costmapメッセージの時刻で、致死セルを次に分類する（`present_margin` 0.30 m、`recent_window` 0.6 s）。
  - present: 現在の障害物外形 + 0.30 m 以内
  - recent: 直近0.6 s以内の外形 + 0.30 m 以内（移動体のセンサー・costmap遅れ分）
  - residual: それ以外（消えるべきなのに残っているコスト）
- 歩行者・車の寸法は代表値（推定）。シミュレーションのVLP-16はノイズσ0.01 m、最大30 m、水平720点で、
  実機の反射率・欠測・路面形状は再現していない。
- 可視化ノードは自前で計算した `/scan3` と実際の `/scan3` を同じ時刻印で比較し、不一致ビン数をログに出す。

## 試験結果（2026-11-09、ヘッドレス、コミット 0abce7a + 本変更）

記録: `reports/velodyne_clearing_study_20261109/baseline_run/`（`timeline.csv`, `summary.json`, `events.jsonl`, ノードログ抜粋）

- 再現性: 可視化ノードの `/scan3` 再計算は実 `/scan3` と一致（比較フレーム中の最大不一致0ビン）。
- 障害物なしの平地: `/scan3` は全720方向が `+Inf`（消去レイ0本）。消去候補（生距離の最大）は `+Inf` が0.1%のみ。
  選択リングの点は全て採用上限外（採用0点 / 除外4314点）。

| フェーズ（シミュレーション時刻順） | 残留セル 開始/最大/終了 | 所見 |
|---|---|---|
| 歩行者 (2.5, 0) 設置→撤去後10 s | 0 → 10 | 撤去後も10セル残留、消えない |
| 歩行者 (6.0, 1.0) 設置→撤去後10 s | 10 → 19 | 同様に残留 |
| 歩行者 x=4 m 横断 1.3 m/s →撤去後 | 19 → 185 | 横断の軌跡が帯状に残留 |
| 自動車 x=9 m 横断 4 m/s →撤去後 | 185 → 426 | 車の軌跡が残留 |
| 低い箱 0.5 m 高 (1.0, −0.6) | 426 → 460 | 残留 |
| 参照壁 x=8 m を設置＋歩行者 (3, 0) | 426 → 260 | 壁でリング8が反射し、壁の角度範囲の過去の残留が消去された |
| 壁を残して歩行者を撤去 | 260 → 260 | 歩行者のセルは消去された（残留は増えない）＝屋内相当では消える |
| 壁を撤去 | 260 → 667 | 壁のセルが丸ごと残留 |

結論（シミュレーション上で確認できたこと）: 反射物のない開けた場所では、障害物撤去後に `/scan3` が `+Inf` となり、
Velodyneレイヤーのコストは時間経過では消えない。背後に壁がある場合は消える。
実機でのコスト残留がこの仕組みだけによるか（TF・自己位置推定・姿勢変化の寄与）は未検証。

## 坂道試験

```bash
# 坂道を可視化（Gazebo GUI付き）。slope_degで勾配[deg]を指定
ros2 launch robotbase_sim velodyne_slope_study.launch.py slope_deg:=10 gui:=true
# 往復させ続ける
ros2 launch robotbase_sim velodyne_slope_study.launch.py slope_deg:=10 gui:=true loop:=true
```

- コース（`scripts/generate_slope_world.py` が起動時に生成）: 平地 0〜4 m → 上り坂（高さ0.8 m）→ 頂上の平地5 m →
  下り坂 → 平地6 m。幅6 m。試験用の形状で、実在の坂の測量値ではない。
- ロボットは**運動学的に移動**（0.4 m/s相当）。`velodyne_slope_drive.py` が20 msごとに `set_pose` で姿勢を与える。
  z＝車輪下の路面高さ、pitch＝前後キャスター接地点を結ぶ弦の傾き（サスペンション付き車体の近似。実機の遷移形状は未確認）。
  - 物理走行をやめた理由: 共通モデル（駆動輪2＋剛体キャスター2）は、平地→上り坂の凹んだ角で前後キャスターが車体を支え、
    駆動輪が浮いて空転・停止した（6°で確認）。キャスターの持ち上げ・ばね化はピッチ誤差が大きく不採用。
  - 坂道試験用に起動時生成するモデルだけ、collision・重力・DiffDriveを外し、`OdometryPublisher`（2D、真値由来）でodom/TFを出す。
    `models/robotbase.sdf` は変更していない。`sim.launch.py` に `model_file` 引数（既定は従来のモデル）を追加。
  - odomは水平距離の真値で、実機の車輪odom（斜面に沿った距離）とは坂で最大3%（14°）程度異なる。
- 障害物は置かないため、costmapの致死セルは全て地面（坂面・平地）の誤登録。
- 真値: 指令姿勢と、Gazeboの `/world/velodyne_slope_study/dynamic_pose/info`（読み戻し）。両者とIMUのpitchは一致を確認。
- 可視化ノードは `use_imu_tilt: true`。地面交点は「ロボット接地面をIMUのpitchで傾けた平面」として描くため、
  坂の始まり・終わり（前方の地形が変わる所）では実際の交点とずれる。点群（採用＝緑／除外＝橙）が実測。
- 記録: `timeline.csv`（指令x/z/pitch、真値、IMU pitch、致死セル数、前方最短距離、/scan3の+Inf率、リング別採用数）、`summary.json`。

### 坂道試験結果（2026-11-09、ヘッドレス、障害物なし）

記録: `reports/velodyne_clearing_study_20261109/slope_runs/`。ピッチは機首上げを正。

| 勾配 | 致死セル最大（区間別: 手前平地 / 上り / 頂上 / 下り / 下りた後） | ロボット前方の最短致死セル | 最初の誤登録 |
|---|---|---|---|
| 6°（10.5%） | 0 / 0 / 0 / 0 / 0 | なし | なし |
| 10°（17.6%） | 29 / 58 / 45 / 93 / 107 | 0.33〜0.35 m（上り・下りた後） | 坂の手前0.3 mの平地（ロボット水平）でリング6が3.4 m先の坂面を登録 |
| 14°（24.9%） | 204 / 549 / 536 / 692 / 1134 | 0.36 m | 坂の1.2 m手前の平地で、リング6が3.4 m先を登録 |
| 18°（32.5%） | 359 / 827 / 811 / 1022 / 1449 | 0.33 m | 坂の1.7 m手前の平地で、リング6が3.4 m先を登録 |

10°で `/scan3` を構成した点のリング・方向（`deg10_ring_probe/ring_probe.json`）:

| 区間 | 主な採用点 | 解釈 |
|---|---|---|
| 坂の手前（水平） | リング6 前方 3.46 m | 前方の上り坂面がリング6の上限3.5 m内に入る（地形起因、IMU補正では防げない） |
| 上り（機首上げ10°） | リング8 後方 中央値8.0 m、側方15 m | +1°のリング8が後方では−9°となり、後ろの平地・坂面に当たる。リング8の上限100 mのため採用 |
| 下り（機首下げ10°） | リング8 前方 中央値7.9 m、側方15 m | リング8が前方で下向きとなり、坂下の平地に当たる |

結論（シミュレーション上で確認できたこと）:
- 6°（10.5%）では地面の誤登録は発生しなかった。10°（17.6%）以上で発生し、勾配とともに増えた。
- 原因は2系統: (1) 上限100 mのリング8が機体の傾きで下向きになり地面を登録する、(2) 機体が水平でも前方の上り坂がリング5・6の上限内に入る。
- 誤登録セルはロボットから0.33 m（footprint端0.30 mのすぐ外）まで現れ、Nav2では走行を妨げる可能性がある。
- 実機の坂・車体の傾き方（サスペンション、遷移形状）での発生は未検証。

## 未実施・既知の制限

- RViz表示は画面で目視確認していない（ヘッドレスで各トピックの配信内容のみ確認）。
- Hokuyoはシミュレーションモデルに無く、レイヤー分離の影響はこの環境では評価していない。
- 消去専用スキャン（`clear_candidate`）をNav2に入れた比較は未実施（次の段階）。
- 坂道試験は運動学的移動で、車輪の滑り・振動・実機サスペンションは含まない。IMU補正（姿勢でリング上限を変える等）の効果は未評価。
