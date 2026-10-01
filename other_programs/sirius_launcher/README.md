# Robot ROS 2 Launch Manager

移植元との互換性でディレクトリ名は `sirius_launcher` のままですが、公開エントリポイントとUIは機体名設定に対応した汎用ランチャーです。現在の表示名は「ココちゃん」です。

## 起動

```bash
cd ~/robotbase_ws/other_programs/sirius_launcher
python3 robot_launcher.py
```

Bash設定を読み込んでいれば `koko_launcher` でも起動できます。デスクトップショートカットは次で再作成します。

```bash
~/robotbase_ws/bash/install_launcher_shortcut.sh
```

## 設定

表示名と通信分離設定の正本は `~/robotbase_ws/robot.env` です。

```bash
ROBOTBASE_DISPLAY_NAME="ココちゃん"
ROBOTBASE_ID="koko"
ROBOTBASE_ROS_DOMAIN_ID="57"
ROBOTBASE_SIM_ROS_DOMAIN_ID="58"
ROBOTBASE_GZ_PARTITION="koko"
ROBOTBASE_TF_PREFIX="robot"
```

表示名を変更した後はランチャーを再起動し、デスクトップショートカットのインストーラをもう一度実行してください。`ROBOTBASE_ID` はエイリアス名とプロセス管理IDに関係するため、通常は変更しません。

ランチャーは `robotbase_ws/bash/bash_alias2.sh` だけを読みます。`sirius_jazzy_ws` へフォールバックしないため、両WSが同じPCにあってもボタンが入れ替わりません。

## 通信とプロセスの分離

- 実機の子プロセス: `ROS_DOMAIN_ID=57`
- シミュレーションの子プロセス: `ROS_DOMAIN_ID=58`
- Gazebo Transport: `GZ_PARTITION=koko`
- TF prefix: `ROBOTBASE_TF_PREFIX=robot`
- PID/ログ: `/tmp/koko_launcher_*`
- Terminatorタブ: `[ココちゃん] koko_*`

ランチャーは子ターミナルが `.bashrc` を読み、シリウス設定へ戻った後にも上記の値を再適用します。

## エイリアスからボタンを作る規則

`bash/bash_alias2.sh` の次のコメントを利用します。

```bash
# PRESET: 地図生成（シミュレーション）
# PRESET_ITEMS: koko_sim,koko_slamtoolbox_sim,koko_rviz_sim

# PRESET: 自律移動（シミュレーション）
# PRESET_ITEMS: koko_sim,koko_nav2_sim_map,koko_rviz_sim

# GROUP: シミュレーション
# 説明文
alias koko_slamtoolbox_sim='koko_src && ros2 launch robotbase_sim mapping.launch.py'
```

- `# GROUP:` がUI内のグループ名になる
- alias直前のコメントが説明になる
- `# PRESET:` と `# PRESET_ITEMS:` が上部ボタンになる
- `koko_src` と `koko_env` はGUI内で明示的な環境設定へ展開される
- インストール用aliasとランチャー自身はボタンから除外される

タブは5つです。SLAM、シミュレーションNav2、シミュレーション用ウェイポイント操作は「シミュレーション」タブへ配置します。実機用ウェイポイント操作は「ナビゲーション」タブ、Foxglove Bridgeは「ユーティリティ」タブです。外部連携タブは削除済みで、LLMとZED/SAM3関連aliasも読み込み対象に含めません。

## テスト

```bash
cd ~/robotbase_ws
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=other_programs/sirius_launcher \
pytest -q other_programs/sirius_launcher/tests
```
