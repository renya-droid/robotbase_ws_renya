# 中之島エレベーター（ATC4 → ATC6）

出発地図は `atc4.yaml`、切替先は `atc6.yaml`。ドアは画像左側の細い通路の左端です（利用者確認済み）。細い通路と先端の丸い部分を合わせて、10頂点の凹多角形でかごを表します。ドアからかご内側は、両地図とも map の +X 方向です。

| 地図 | ドア左端 (x, y) [m] | ドア右端 (x, y) [m] |
| --- | --- | --- |
| ATC4 / 出発 | (226.601, 13.461) | (226.601, 12.661) |
| ATC6 / 到着 | (1.606, 0.374) | (1.606, -0.426) |

`geometry_review.png` のオレンジ線がドア、水色線がかご外周です。`geometry_pixels.yaml` に元の画像ピクセル座標を保存しています。座標は各地図YAMLの解像度・原点を使い、画像の上下反転を含めてピクセル中心から変換しました。PGM画像は変更していません。

`switch_map_before_exit: true` により、退出前にATC6へ切り替えます。既存の `arrival_exit_mode: map_only` と手動承認設定を維持しています。ROS domain は既存の実機設定から54を継承しています。

地図画像からの読み取りであり、現地実測は未確認のため `measurements_reviewed: false` です。通常起動には既存の確認条件が適用されます。座標案は以下で検証できます。

```bash
ELEVATOR_PROFILE=nakanoshima python3 ~/kuwa_ws2/tools/configure.py draft-check
```

形状のくぼみに対応するため、`arrival_geometry.py` は単純な凹多角形も受け付けます。自己交差・重複辺の退化形状は拒否し、車体の頂点だけでなく辺もかご壁を横切らないか確認します。
