# SiriusとココちゃんのNav2比較（2026-10-22）

これは旧展開版との比較です。後から取得されたクローン版との最新比較は [cloned_comparison.md](cloned_comparison.md) を参照してください。

同じMPPI実装を使用していますが、設定・評価項目・BTは同じではありません。
大きな方位差への対応を見るうえでは、低速カット、前進優先、予測時間、Spin復帰の違いが重要です。
以下はファイルと起動処理の比較で、設定変更・ノード再起動は行っていません。

## 比較対象

`/home/ren/sirius_jazzy_ws` は存在しなかったため、次の展開済みワークスペースを調べました。

`/home/ren/sirius_jazzy_ws-20261002T175611Z-1-001/sirius_jazzy_ws`

- Sirius実機: `params/nav2_params.yaml`
- Sirius sim: `params/nav2_params_sim.yaml`
- Sirius simの `nav2_params_sim_no_semantic.yaml` と `nav2_params_sim_slam.yaml` も確認。今回のMPPI・平滑化・BT設定は通常simと同じ。
- ココちゃん実機: `params/real/nav2.yaml`
- ココちゃんsim: `params/sim/nav2.yaml`

Siriusの起動スクリプトは実機・simで上記ファイルを `nav2_bringup bringup_launch.py` に渡します。
simはセマンティックレイヤーの選択によって通常版とno_semantic版を切り替えます。
現在のSirius稼働プロセスや実行時パラメータ、当時の実機bagの実効設定まで確認したものではありません。

## 大きな方位差への姿勢合わせ

**両方とも `FollowPath.plugin = nav2_mppi_controller::MPPIController` を直接指定しており、Rotation Shimによる走行前の姿勢合わせはありません。**
`primary_controller`・`angular_dist_threshold` などのShim設定もありません。
SiriusのソースツリーにRotation Shimパッケージが存在することと、設定で使用していることは別です。

`PathAngleCritic` は候補軌道の評価、`GoalAngleCritic` はゴール付近の方位評価であり、
それだけで「最初にその場旋回を完了してから前進」という独立した動作にはなりません。

Siriusの既定BTには失敗後の `Spin spin_dist=1.57`（約90度）があります。
ココちゃんはSpinプラグイン自体をロードしますが、起動時に指定する独自BTではそのSpin呼び出しを省いています。
**Siriusには復帰時の旋回があるが、経路の方位差へ事前に合わせるRotation Shimは両方にない**、という違いです。

## MPPI・速度の主要な違い

|設定|Sirius実機|Sirius sim|ココちゃん実機・sim|
|---|---:|---:|---:|
|予測 `time_steps × model_dt`|60×0.1 = 6秒|同左|28×0.1 = 2.8秒|
|`prune_distance`|6m|6m|3m|
|前進上限 / 角速度上限|0.9m/s / 0.9rad/s|同左|同左|
|前進・角加速度上限|0.9m/s² / 1.5rad/s²|同左|同左|
|後退下限 `vx_min`|−0.6m/s|−0.6m/s|−0.3m/s|
|速度探索ノイズ `vx_std`|0.25|0.25|0.30|
|`regenerate_noises`|true|true|false|
|`visualize`|true|true|false|
|前進優先重み `PreferForwardCritic`|10|15|40|
|前進優先のゴール近傍無効化距離|2.5m|2.5m|0.5m|
|低速評価 `VelocityDeadbandCritic`|使用しない|使用しない|重み35、前進0.1m/s未満へペナルティ|
|旋回評価 `TwirlingCritic`|重み3|重み3|使用しない|
|`PathAngleCritic.cost_weight`|2|2|5|
|同 `max_angle_to_furthest`|1.2rad（約69度）|同左|0.5rad（約29度）|
|同 `offset_from_furthest`|6|6|4|
|`PathFollowCritic.offset_from_furthest`|5|5|11|
|`PathAlignCritic.max_path_occupancy_ratio`|0.20|0.20|0.10|
|`CostCritic.cost_weight`|10|16|4|
|同 `near_collision_cost`|150|120|200|
|同 `critical_cost`|250|250|300|
|`ConstraintCritic.cost_weight`|10|10|6|
|`GoalAngleCritic.cost_weight`|1|1|3|
|速度平滑化 `deadband_velocity`|[0,0,0]|[0,0,0]|[0.1,0,0]|
|controllerのodom前進速度のゼロ扱い閾値|0.05m/s|0.05m/s|0.10m/s|
|controllerのodom角速度のゼロ扱い閾値|0.05rad/s|0.05rad/s|0.001rad/s|
|ゴール許容位置 / 方位|0.5m / 0.5rad|同左|0.25m / 0.25rad|
|進捗判定|30秒で0.3m|同左|30秒で0.2m|

バッチ数2000、DiffDrive、temperature=0.30、gamma=0.050、iteration_count=1、
角速度探索ノイズ0.30、制御周波数10Hzは共通です。
MPPIの比較対象C++ヘッダ・ソースは同一でした。実行中の共有ライブラリの同一性を検証したものではありません。

解釈上の注意:

- `min_x_velocity_threshold` は入力odomの速度をゼロ扱いする閾値で、出力指令を切る `deadband_velocity` とは別です。
- `VelocityDeadbandCritic` は低速を禁止する制約ではなく、低速の候補へコストを足します。重み35でも0.1m/s未満の出力はあり得ます。これを平滑化側がゼロにすると、今回観測した0.084→0m/sの停滞と整合します。
- `PreferForwardCritic` は後退にコストを加えます。ココちゃんは重みが高いうえ、ゴールへ0.5mまで近づかない限り有効。Siriusは2.5m以内では無効です。
- `TwirlingCritic` は無駄な旋回へコストを加えるもので、「向き直りを促す」設定ではありません。
- 重みは他のcriticや予測時間と組み合わさるので、数値の大小だけで旋回の良否・安全性は断定できません。
- `prune_distance=6m` でも、ローカルコストマップ外はMPPIで切り出されるため、常に6m先まで使えるわけではありません。

## BTの違い

SiriusはYAMLで既定BTを上書きせず、Navigatorのソースにある既定ファイルを使用する設定です。
ココちゃんは `src/robotbase_bringup/launch/nav2.launch.py` が実機・sim共通で独自BTのパスを上書きします。
RVizから送っても、その既定BTが使われます（goalで個別BTを指定した場合を除く）。

|項目|Siriusの既定BT|ココちゃんの起動BT|
|---|---|---|
|ToPose再計画|1Hz|1Hz|
|ThroughPoses再計画|0.333Hz|0.333Hz|
|失敗時の一般復帰|地図クリア→Spin約90度→Wait5秒→BackUp|地図クリア→Wait5秒→BackUp|
|BackUp|0.30m、0.15m/s|同じ|
|全体の再試行上限|6|6|
|経路の急な伸長への待機|通常既定BTにはない|3m未満の経路が3倍超に伸びたときCancelControl→Wait3秒|
|ThroughPosesの通過点削除|`RemovePassedGoals radius=0.7` を再計画側で実行|独自 `RemoveTraversedGoals` をBT毎tickで実行|

ココちゃんの独自ウェイポイントクライアントには、90度以上の曲がりや折り返し付近で
ThroughPosesの送信区間を分ける処理もあります。これはRVizからの直接送信では通りません。
BT内の通過点削除や、MPPI内部の参照範囲の切り出しは、それとは別の処理です。

## コストマップの違い

- ココちゃんの外形は前後各0.30m・左右各0.29m。Siriusは前0.50m・後0.70m、左右はglobal各0.35m・local各0.30m。旋回時の衝突条件が異なります。
- ココちゃんlocalは6×6m。Siriusは実機6×6m、sim7×7m。
- ココちゃんglobalは全体地図、未知領域を追跡。Siriusは50×50mのrolling window、未知領域を追跡しない設定。
- ココちゃん実機はVLP/HokuyoのObstacleLayerを分離。Sirius localは3D点群のSTVLも使用。simにはセマンティックレイヤーの有無の分岐がある。
- global inflation radiusはココちゃん0.70m、Sirius0.75m。localはココちゃん0.70m、Sirius実機0.70m・sim0.80m。localのcost_scaling_factorはココちゃん5、Sirius10。

同じMPPI数値に揃えるだけで、同じ入力コストマップ・同じ挙動になるわけではありません。

## まず比較すべきもの

今回の停滞との対応が最も直接的なのは、**Siriusにはない速度平滑化の0.1m/sカット**です。
次に、前進優先と低速ペナルティ、予測時間・参照距離、失敗後のSpin有無を一つずつ比較するのがよいと考えます。
Siriusに「大きな方位差なら先に向き直る特別なShimがある」という差ではありません。

全パラメータの差分は [parameter_differences.csv](parameter_differences.csv) に保存。
`未指定` はファイル中に明示されていない意味で、無効・ゼロとは限りません。
