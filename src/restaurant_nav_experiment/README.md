# 配膳ロボット × レストラン人物シミュレーション実験

「人の位置と、着席しているか歩いているかが分かれば、配膳ロボットの走行は改善するか」を確かめるための実験一式です。位置はシミュレータの正解値をそのまま使います（完璧な外部カメラを仮定した**効果の上限**の測定）。カメラの誤差・遅延・死角は入っていません。

## 比較する条件

| 条件 | Global Costmapに追加するもの | 確かめること |
| --- | --- | --- |
| `B0` | なし（LiDARと静的地図のみ） | 基準 |
| `B1` | 全員を同じ円（`external_camera_people_layer`、半径1.0 m、最大コスト220） | 位置が分かる効果 |
| `P` | 歩行者だけ円（半径1.0 m、220）。着席者は既定で追加しない（`seated_radius:=0.4` などで別Layerを追加可） | 状態が分かる効果（B1との差） |

Layerのプラグイン本体（C++）は変更していません。条件Pはプラグインを2つ（歩行者用・着席者用）並べて別トピックを購読させます。Local Costmapと制御器は全条件で同じです。

## 構成

- `launch/experiment.launch.py`：ロボットのシミュレーション、Nav2（条件別の一時YAML）、人物ノード、橋渡しノード、配膳実行ノードを起動
- `launch/robot_sim.launch.py`：`robotbase_sim/launch/sim.launch.py` のコピー。変更は2点だけ（先頭に注記）
  1. `twist_mux` がこのPCにないため、同じ `params/sim/twist_mux.yaml` を読む `twist_mux_lite` を使う
  2. DiffDriveのオドメトリTFの代わりに、Gazeboの正解位置からTFを出す（`ground_truth_tf`）
- `people_bridge`：人物のsnapshotを `/external_camera/people`（全員）・`people_walking`・`people_seated`（map座標）に分けて配信し、ロボット位置を `/restaurant/robot_odom` で人物ノードへ返す
- `serving_runner`：基地 → 席 → 基地 を繰り返し、`legs.csv`・`samples.csv`・`summary.json` を書く
- `scripts/generate_assets.py`：ワールド・地図・基地・配膳先を `generated/` に生成（入力ファイルは変更しない）
- `scripts/run_condition.sh`・`run_pilot.sh`・`summarize.py`：実行と集計

## 座標系

`map` の原点はロボットの基地（出現位置、world (7.45, 3.1)、向き0）です。`map = world − 基地`。Nav2 は `localization:=static`（map→odom 恒等）で、odom→base_footprint は Gazebo の正解位置です。比べたいのは経路計画の違いなので、自己位置推定の誤差を入れていません。

## 実行

```bash
cd ~/robotbase_ws_renya
source /opt/ros/jazzy/setup.bash && source install/setup.bash
# 1条件（専用Domain 94、ヘッドレス、終わると自分のプロセスだけ片付ける）
src/restaurant_nav_experiment/scripts/run_condition.sh P high 42 artifacts/restaurant_nav/try_P
# 条件 × seed をまとめて
src/restaurant_nav_experiment/scripts/run_pilot.sh artifacts/restaurant_nav/pilot "B0 B1 P" "42 11 22"
```

GUIで見る場合は `run_condition.sh` の末尾に `gui:=true` を付けます。

## 測る指標（`legs.csv`、1行 = 1区間）

所要時間・結果（SUCCEEDED/FAILED/TIMEOUT）、走行距離、ロボット停止時間（|v|<0.05 m/s）、ロボット外形から人の円柱面までの最小距離（全員・歩行者）、0.2 m未満への接近回数、接触サンプル数（距離<0）、ロボット中心から1.2 m以内で止まっていた人の延べ時間（列待ち・意図的な待ちは除く）、到着時にその席に人が座っていたか。失敗した区間の後はロボットを基地へテレポートしてCostmapを消去し（`summary.json` の `resets`）、1回の失敗が後の区間を巻き込まないようにしています。

## 仮定・制約（結果を読むときの注意）

- 人物の位置は正解値（効果の上限）。カメラ誤差モデルは次段階。
- 人はロボットを向きに沿った3点（中心・前0.2 m・後0.25 m）として0.66 m離れ、近づかれると止まるだけで、よけない。人が譲る確率は未パラメータ化。
- 配膳先は、ロボット外形が家具から0.10 m以上離れ、かつその場で旋回できる（中心から家具まで0.63 m以上）位置のうち、椅子に最も近い位置（自動選択、向きは客の方）。18席のうち中央の菱形テーブルの6席は条件を満たす位置がなく対象外（12席）。椅子の方を向いて家具まで0.10 mの位置では旋回できず、戻りの区間がタイムアウトしたため旋回条件を加えた。
- 基地はキッチンカウンター東側 world (7.45, 3.1)、旋回できる余裕のある最も西の位置。厨房東側の通路（x≈7〜8）の中にあり、人の通り道でもある。テーブル前の停止時間5秒も仮定。
- 配膳先の順番は seed で決まる固定の列で全条件共通。人がロボットに反応するので、条件間で人の動きは分岐する（到着時の着席有無を記録）。
- 人物シナリオは一斉入店から約400秒で空になる一山型。`start_sim`/`stop_sim`（Gazebo時刻）で配膳する時間帯を決める。
- 負荷対策として、Nav2 BTの `default_server_timeout` を20 ms→200 ms（全条件同じ）、人物の位置反映をCLIからROSサービス（`pose_transport:=ros`）に変更。
- **Local Costmapに静的地図のLayerを追加**（全条件同じ、`local_static:=false` で元の構成）。元の構成では、高さ0.72 m・下向き15°のLiDARから高さ0.45 mの椅子が約1 m以内で見えず、MPPIが旋回時にロボットの角を椅子へ当てて「Failed to make progress」で戻りの区間が失敗した（無人ワールドで12区間中4区間）。実機でも起こりうる構成上の注意点。
- 地面：元のワールドは床の上面がz=0.10で、ロボットがz=0.01に出現すると埋まるため、生成ワールドでは床を0.10下げている（家具は元からz=0に立っている）。
