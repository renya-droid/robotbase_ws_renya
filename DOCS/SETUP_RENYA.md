# このPCの環境構築状況（2026-10-01）

現在のワークスペースは `/home/ren/robotbase_ws_renya` です。

## 完了した項目

- ROS 2 Jazzy、colcon、ランチャー用PySide6、Terminator、xterm、Gazeboコマンドの存在を確認。
- 59パッケージを `colcon build --symlink-install --executor sequential --allow-overriding nav2_costmap_2d` でビルド済み。コンパイラ警告はあるがビルド失敗なし。
- ランチャーの既存テスト4件が成功。起動補助スクリプトのBash構文も検査済み。
- デスクトップとアプリメニューの `koko_launcher.desktop` をこのワークスペースへ切り替え。
- `~/.bashrc` のココちゃん用alias読み込み先をこのワークスペースへ切り替え。
- ランチャーの子端末で、このフォルダのaliasを明示的に再読み込み。
- 起動・地図・記録用スクリプトの旧 `robotbase_ws` 参照を切り替え。
- `koko_env` は旧 `robotbase_ws` と `sirius_jazzy_ws` の検索パスを除去。
- amentインデックスで実機bringup、Roboteq、Hokuyo、シミュレーションの参照先が、このワークスペースの `install/` 配下であることを確認。
- `foxglove_bridge` は `/opt/ros/jazzy/` に導入済み。
- waypointのmanifestから解決不能な `ament_python` 依存指定を削除。ビルド種別の `ament_python` は維持。該当パッケージの再ビルドと `rosdep check --from-paths src --ignore-src` は成功。

## 現在の設定

正本は `robot.env` と `params/real/` です。

| 項目 | 値 |
|---|---|
| 実機 / シミュレーションDomain | 54 / 67 |
| TF接頭辞 | `robot` |
| Roboteq / IMU | `/dev/roboteq` / `/dev/wt905` |
| VLP-16 | `192.168.1.201`、UDP 2368、`/scan3` |
| 足元Hokuyo | `192.168.0.9`、TCP 10940、`/hokuyo_scan` |
| odom | 車輪累積カウントの差分から `/odom` を計算。EKFは車輪前進速度とIMUヨー角速度を融合して `/odom/filtered` を配信 |
| 標準回復 | Spinを呼ばないBT。Spinプラグインの登録は残る |

## 実機接続時に残る確認

確認時点では `/dev/roboteq` と `/dev/wt905` は見つからず、`/etc/udev/rules.d/` にRoboteq用ruleも見つかりませんでした。実機の接続・走行と、このPCでのGazebo起動は未確認です。

Roboteq用ruleは同梱しています。対象コントローラのVID/PIDを確認してから [実機PCへの移行手順](REAL_PC_MIGRATION.md#5-roboteq) に従って適用してください。IMUはUSBのVID/PIDまたはserialの実測が必要なので、接続前にruleを推測して作成しません。シリアルアクセス権限も接続後に確認してください。

LiDAR接続NICのIP、VLP-16の送信先IP、Hokuyoの疎通、取付TF、各トピックの更新を実機で確認してください。この作業中はネットワーク設定変更やモータ起動を行っていません。

## 起動と再ビルド

開いていたランチャーは閉じて、デスクトップから起動し直してください。端末は新しく開くか、次を実行します。

```bash
source ~/robotbase_ws_renya/bash/bash_alias1.sh
source ~/robotbase_ws_renya/bash/bash_alias2.sh
koko_launcher
```

現在のPCで全パッケージを再ビルドする必要はありません。ソース変更後は、必要なパッケージだけを再ビルドできます。

```bash
cd ~/robotbase_ws_renya
source bash/activate_koko_env.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential --packages-select <変更したパッケージ>
```

別PCに移す場合は [実機PCへの移行手順](REAL_PC_MIGRATION.md) の依存導入とビルドから行ってください。
