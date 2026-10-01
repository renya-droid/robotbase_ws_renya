# Hokuyo 2D LiDAR

## 構成

- ドライバ: `urg_node2`
- 依存ライブラリ: `laser_proc` 1.0.2
- 接続: Ethernet `${ROBOTBASE_HOKUYO_IP}:${ROBOTBASE_HOKUYO_PORT}`（現在の実機 `192.168.0.9:10940`）
- 出力: `/hokuyo_scan` (`sensor_msgs/msg/LaserScan`)
- `frame_id`: `robot.env` の `ROBOTBASE_TF_PREFIX` + `/hokuyo_link`
- 取付位置: `base_footprint` 基準で `x=0.196 m, y=0 m, z=0.11 m`
- 計測角度: センサー正面を0°とした `-90°〜+90°`（前方180°）

`urg_node2`は`/home/koko-chan-nuc/sirius_jazzy_ws`で使用しているものを移植した。HokuyoのTFは`robotbase.urdf`の`hokuyo_joint`で定義する。センサーを上下反転して取り付ける場合は `rpy` も実装に合わせる。

## ビルドと起動

```bash
cd ~/robotbase_ws
koko_build
source install/setup.bash

ping -c 3 "${ROBOTBASE_HOKUYO_IP}"
koko_hokuyo
```

現在のPC有線LANはVelodyne用の`192.168.1.100/24`とHokuyo用の`192.168.0.100/24`を持つ。Hokuyo実機は`192.168.0.9/24`のため、VelodyneとHokuyoを同じ有線NIC/スイッチで使う場合は両方のPC側アドレスが必要。`ip address add`で追加したアドレスは再起動で消えるため、NetworkManagerで永続化するか、次回起動時に再追加する。

- HokuyoのIPを`192.168.1.x`へ変更し、`robot.env`の`ROBOTBASE_HOKUYO_IP`も合わせる。
- PCの有線NICへ`192.168.0.x/24`のセカンダリIPを追加する。

ランチャーの「実機基本」プリセットからも起動できる。

## 確認

```bash
koko_src
ros2 lifecycle get /urg_node2
ros2 topic info /hokuyo_scan
ros2 topic hz /hokuyo_scan
ros2 topic echo /hokuyo_scan --once
```

正常時は`urg_node2`が`active`になり、`/hokuyo_scan`のheaderに`robot/hokuyo_link`が入る。次のコマンドで取付TFも確認できる。

```bash
ros2 run tf2_ros tf2_echo "${ROBOTBASE_TF_PREFIX}/base_footprint" "${ROBOTBASE_TF_PREFIX}/hokuyo_link"
```

Nav2のglobal/local costmapは、VLP-16から作る`/scan3`と足元の`/hokuyo_scan`の両方を使う。それぞれを独立したObstacleLayerに入れ、片方のclearingがもう一方で検出した障害物を消さないようにする。SLAM ToolboxとAMCLの自己位置推定入力は`/scan3`を継続使用する。

> [!CAUTION]
> 実機Nav2は`/scan3`と`/hokuyo_scan`の両方を障害物回避に使う。両トピックの更新を確認するまで自律走行を開始しないこと。

RVizは前回保存した表示設定を再利用する。Hokuyo表示が追加されない場合は、次を1回実行してテンプレートから再生成する。

```bash
koko_src
ros2 launch robotbase_bringup rviz.launch.py use_sim_time:=false publish_description:=false tf_prefix:="${ROBOTBASE_TF_PREFIX}" reset_config:=true
```
