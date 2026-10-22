# 0920-nakanoshima 地図から生成したワールド

ランチャーの `koko_sim` の選択画面で
**0920-nakanoshima（実測地図・占有セルから生成）** を選べます。
すでに選択画面を開いている場合はいったん閉じて開き直してください。

直接起動する場合:

```bash
koko_sim --world-id map_0920_nakanoshima
```

Nav2は別端末で対応地図を指定して起動します。

```bash
koko_nav2_sim_map --world-current
```

RVizも別端末で起動します。

```bash
koko_rviz_sim
```

## 生成条件

- 元地図: `maps_waypoints/maps/0920-nakanoshima.yaml` / `0920-nakanoshima.pgm`
- ワールドID: `map_0920_nakanoshima`
- SDF: `src/robotbase_sim/worlds/map_0920_nakanoshima.sdf`
- 対応地図: `src/robotbase_sim/maps/map_0920_nakanoshima/map.yaml`
- スポーン: `(0, 0, 0)`。既知の空き領域の余裕は約1.82m。
- 解像度0.05m、画像5586×6649、origin `[-182.75, -233.35000000000002, 0]` を維持。
- 占有137,150セルを高さ2mに押し出し。メッシュ836,672三角形。
- PNGへの変換は可逆で、未知・空き・占有の全画素を元画像のまま保存。

1F-0821と同じ生成方法です。壁・柱・家具・計測ノイズを区別せず、未知領域に壁は補いません。
建物の実際の高さやLiDARの反射特性を復元したものではありません。
地図とワールドの座標が同じなので、対応する `0920-nakanoshima` のウェイポイントを座標変換せず利用できます。
自律走行で全経路を走破したことは確認していません。

## 再生成

```bash
python3 src/robotbase_sim/scripts/generate_map_world.py \
  --map maps_waypoints/maps/0920-nakanoshima.yaml \
  --world-id map_0920_nakanoshima
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robotbase_sim --symlink-install
```

## 検証結果（2026-10-22）

- 地図画素・origin・縮尺・スポーン余裕・既存ワールド連携などの関連テスト10件成功。
- SDF構文チェック、パッケージビルド成功。
- ランチャーの選択画面に項目が入り、選択すると正しいワールドIDが起動コマンドに付くことを確認。
- 使用中のsimとは別のROS Domain 220で、インストール済みSDFからロボットを起動。
- `/clock`・`/odom`・`/scan3` が配信。LiDAR720点中302点が有限値、最短距離約1.80m。
- 初期位置のオドメトリは原点を維持。試験用simは確認後に終了。
