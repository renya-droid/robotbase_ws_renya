# ココちゃん パラメータ

このフォルダをココちゃん固有パラメータの正本とする。`src/navigation2`、`src/slam_toolbox`、`src/velodyne` にある上流パッケージの既定ファイルは編集しない。

- `real/`: 実機用。Nav2、SLAM Toolbox、twist_mux、Roboteq、IMU、EKF、Velodyne。
- `sim/`: Gazebo用。Nav2、SLAM Toolbox、twist_mux、点群からLaserScanへの変換、停止時のゼロ速度発行。
- `common/`: 実機・シミュレーション共通。キーボード手動操作とFoxglove Bridge。

Nav2・SLAM Toolbox・twist_muxは、差分を直接比較して個別に調整できるよう実機用とシミュレーション用を別ファイルにしている。

Nav2標準Waypoint Followerの設定も`real/nav2.yaml`と`sim/nav2.yaml`内の`waypoint_follower`に分けている。SIRIUS互換YAMLを使う独自クライアント本体は`src/robotbase_waypoint/`、操作手順は[`DOCS/WAYPOINTS.md`](../DOCS/WAYPOINTS.md)を参照する。

Nav2のローカルコントローラは実機・シミュレーションともMPPIを使用する。ココちゃんの長方形footprint全体で障害物との衝突を評価するため、`CostCritic.consider_footprint`を有効にしている。

MPPIには固定の巡航速度パラメータがないため、実機・シミュレーションとも`vx_max`とvelocity smootherの前進上限を`0.60 m/s`にして、これを運用上の基準速度とする。経路進行評価は共通で`PathFollowCritic.cost_weight: 12.0`・`offset_from_furthest: 11`とし、十分な空き領域の直線では上限付近を狙う。

実機は周期的な加速感を抑えるため、`vx_std: 0.25`、`gamma: 0.050`、前進加速度上限`0.60 m/s^2`とする。候補軌道の描画と毎周期のnoise再生成も無効にして、controllerの計算周期と出力を安定させる。シミュレーションは調整内容をRVizで確認できるよう、`vx_std: 0.30`、`gamma: 0.015`、`visualize: true`、`regenerate_noises: true`を維持する。

実機用は、モータが反応しない前後速度`0.10 m/s`未満をMPPIの`VelocityDeadbandCritic`、controllerのodom閾値、velocity smootherのdeadbandへ設定している。シミュレーション用も低速停滞を避けるため、MPPI Criticを0.12 m/s、odom閾値とsmootherを0.10 m/sとし、`sim/keyop.yaml`の直進刻みを0.20 m/sにする。角速度の不感帯は実測値が得られるまで0とする。

実機Roboteqの`speed_scale`はモータ指令側だけの校正値で、現在の作業値は`0.70`である。動的変更はビルド・再起動不要、YAMLへ保存した値は次回driver起動時に読み込まれる。調整手順とodom校正との区別は[構成資料](../DOCS/CONFIGURATION.md#実機の指令速度校正)を参照する。

通常はlaunchの既定値でこのフォルダが使われる。別ファイルを試す場合は、各launchへ `params_file:=...` または `slam_config_file:=...` を渡す。
