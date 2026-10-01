# SLAM Toolbox設定とシリウスとの差分

## 比較対象

シリウス側は、実機用エイリアス `slamtoolbox_real` が実際に指定している次のファイルを比較元とする。

```text
~/sirius_jazzy_ws/params/mapper_params_online_async.yaml
```

ココちゃん側の実機用設定は次のファイルである。

```text
~/robotbase_ws/params/real/slam_toolbox.yaml
```

`sirius_navigation/config/slam_toolbox_params.yaml` やSLAM Toolbox本体の `config/` は、実機用エイリアスが直接使う設定ではないため、この表の比較対象にはしていない。

## 現在の差分

| パラメータ | シリウス実機 | ココちゃん実機 | 意味・影響 |
|---|---:|---:|---|
| `use_sim_time` | 未指定（launchでfalse） | `false` | 実質的な動作は同じ。ココちゃんは設定ファイルにも明記している。 |
| `odom_frame` | `sirius3/odom` | `odom` | ココちゃんはlaunch時にTF接頭辞を加え、標準では `robot/odom` になる。 |
| `base_frame` | `sirius3/base_footprint` | `base_footprint` | ココちゃんはlaunch時に標準で `robot/base_footprint` へ書き換える。 |
| `scan_topic` | `/scan3` | `/scan3` | どちらも複数リングを距離制限付きで合成した独自2.5D scanを使用する。 |
| `throttle_scans` | `0` | `1` | シリウスの0はSLAM Toolbox内部で警告後に1へ補正されるため、実際はどちらも全スキャン処理。ココちゃんは有効値を明記した。 |
| `map_update_interval` | `3.0 s` | `3.0 s` | 同一。OccupancyGrid表示の更新周期。 |
| `min_laser_range` | `1.0 m` | `1.0 m` | 同一。1 m以内の人、車体、ケーブル等を地図から除外する。 |
| `max_laser_range` | `100.0 m` | `100.0 m` | 同一。実環境では遠距離ノイズも通すため要観察。 |
| `transform_timeout` | `0.2 s` | `0.2 s` | 同一。スキャン時刻のTF到着を待つ時間。 |
| `stack_size_to_use` | `100000000` | `100000000` | 同一。大規模ポーズグラフのシリアライズ等に使用。 |
| `minimum_travel_distance` | `0.30 m` | `0.30 m` | 同一。0.3 m移動するごとにスキャン節点を追加する。 |
| `minimum_travel_heading` | `0.30 rad` | `0.30 rad` | 同一。約17.2度旋回するごとにスキャン節点を追加する。 |

機体固有のTF名、時刻設定、`throttle_scans` の正常値表記を除き、Ceres、更新周期、距離範囲、スキャンマッチング、ループクロージャ、相関探索、評価ペナルティはシリウス実機用設定と同一である。

## 地図のゴミとの関係

SLAM Toolboxの調整値はシリウスへ合わせたため、設定差によるゴミの増加要因は減った。ただし、次の機体差は残る。

1. Nav2の実機前進基準速度 `0.60 m/s`、最大角速度 `0.90 rad/s` に対し、VLP-16は10 Hzで1周するため、移動中のスキャン歪みが発生し得る。
2. LiDARの取付高さ・前後位置・ロール・ピッチ・ヨーがシリウスと異なる可能性がある。
3. 車輪径、トレッド幅、エンコーダ、IMU取付方向が異なり、オドメトリ誤差の特性も異なる。

`/scan3` のリング構成と距離上限はシリウスと同じ既定値を明示している。ただし、LiDARの実取付高さや傾きが異なる場合は、同じ距離上限が最適とは限らない。

`min_laser_range: 1.0` に合わせたことで近距離の人や車体周辺は除外される。一方、`max_laser_range: 100.0` に合わせたため、屋内で遠距離反射ノイズが出る場合は再調整候補になる。`map_update_interval` は主にRViz等へ出す地図の更新周期なので、通常はゴミの直接原因ではない。

## 調整するときの方針

- 地図作成中はNav2速度を通常走行より低くする。
- 停止中にもゴミが出る場合は速度ではなく、LiDARのTF、傾き、反射、近距離点を確認する。
- 近い壁も地図へ必要なら、`min_laser_range` を `1.0`から段階的に下げて比較する。ただし人や車体周辺の点も入りやすくなる。
- `minimum_travel_distance` と `minimum_travel_heading` は、速度を落としてから調整する。先に大きくすると地図更新が粗くなる。
- 実機とシミュレーションは別ファイルを維持し、実機で確認した値を無条件にシミュレーションへ反映しない。

## 2D Pose Estimateとの関係

`enable_interactive_mode: true` は、RVizのSLAM Toolboxプラグインからポーズグラフを編集するための設定であり、標準の2D Pose Estimateとは別である。

- mappingモードのSLAM Toolboxは `/initialpose` を購読しない。
- 既存地図＋AMCLモードでは `/initialpose` をAMCLが購読する。
- AMCLで使うときはRVizのFixed Frameを `map` にする。
