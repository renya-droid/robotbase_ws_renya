# Foxglove Bridge

## 構成

ココちゃんのROS 2グラフをFoxgloveへWebSocket配信する。実機とGazeboで同じサーバーを使い、起動時のROS Domainに存在するトピック、TF、サービス、パラメータを公開する。

```text
ROS_DOMAIN_ID=54  # 実機。シミュレーションは58
  └─ koko_foxglove_bridge
       └─ ws://0.0.0.0:8766
            └─ Foxglove Desktop / Web
```

Siriusの標準Foxglove Bridgeはポート8765を使うため、同一PCで同時起動できるようココちゃんは8766を既定値にしている。ココちゃん実機はDomain 54、シミュレーションはDomain 58で分離される。

## 初回導入

各PCで一度だけ実行する。

```bash
sudo apt update
sudo apt install -y ros-jazzy-foxglove-bridge

cd ~/robotbase_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robotbase_bringup
source install/setup.bash
```

## 起動と接続

実機またはシミュレーションのROSノードを起動した後、別ターミナルまたはランチャーの「ユーティリティ」タブから起動する。

```bash
koko_foxglove      # 実機Domain 54
koko_foxglove_sim  # シミュレーションDomain 58
```

起動端末にPCのIPアドレスとポートが表示される。Foxgloveで「Open connection」→「Foxglove WebSocket」を選び、次を指定する。

```text
ws://<robotbase PCのIP>:8766
```

同じPCから接続する場合:

```text
ws://localhost:8766
```

接続できない場合は、PC間が同じネットワークにいることと、UbuntuのファイアウォールでTCP 8766が許可されていることを確認する。

```bash
ss -ltnp | grep 8766
sudo ufw allow 8766/tcp
```

## 設定の正本

- 待受アドレス・ポート: `robot.env`
- topic/service/capability設定: `params/common/foxglove.yaml`
- ROSノード起動: `src/robotbase_bringup/launch/foxglove.launch.py`
- 導入確認・接続先表示: `bash/startup_bash/foxglove_server.sh`
- Bash/UI項目: `bash/bash_alias2.sh` の `koko_foxglove` / `koko_foxglove_sim`

ポート変更例:

```bash
ROBOTBASE_FOXGLOVE_ADDRESS="0.0.0.0"
ROBOTBASE_FOXGLOVE_PORT="8767"
```

変更後はFoxglove Bridgeだけ再起動する。再ビルドは不要。

直接launchする場合:

```bash
ros2 launch robotbase_bringup foxglove.launch.py address:=127.0.0.1 port:=8766
```

## セキュリティ

既定値は`127.0.0.1:8766`で、同じPCのFoxglove Studioからのみ接続できる表示専用構成である。設定は`params/common/foxglove_view_only.yaml`を使い、topic publish、service call、parameter操作を公開しない。

表示専用に近づける場合は、`params/common/foxglove.yaml` の `capabilities` から次を外す。

- `clientPublish`
- `parameters`
- `parametersSubscribe`
- `services`

PC内だけで使う場合は `ROBOTBASE_FOXGLOVE_ADDRESS="127.0.0.1"` に変更する。
