# 既存環境を残してココちゃんランチャー2を導入する

この手順は別PCで実行します。`robotbase_ws`、既存のショートカット、`.bashrc` を編集せず、`robotbase_ws_renya` を別ディレクトリへcloneしてください。

## cloneとビルド

リポジトリのURLはGitHubの「Code」から取得します。

```bash
git clone --recurse-submodules <robotbase_ws_renyaのURL> ~/robotbase_ws_renya
```

Keepoutエディタはprivate submoduleなので、閲覧できるGitHubアカウントで認証してください。
clone済みの場合は、そのWSで `git submodule update --init --recursive` を実行します。

既存の `.bashrc` が読み込まれない子シェルでビルドします。`exit` すると元の端末に戻ります。

```bash
bash --noprofile --norc
cd ~/robotbase_ws_renya
source bash/activate_koko_env.sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -riy
colcon build --symlink-install --executor sequential --allow-overriding nav2_costmap_2d
exit
```

ランチャーに必要なPythonパッケージはPySide6、psutil、PyYAML、Pillow、端末はTerminatorです。

## インストールと起動

```bash
bash ~/robotbase_ws_renya/bash/install_launcher_shortcut.sh
```

デスクトップとアプリ一覧に「ココちゃんランチャー2」が追加されます。
端末からは次で起動できます。`~/.local/bin` がPATHに含まれていれば `koko_launcher2` だけでも起動できます。

```bash
~/.local/bin/koko_launcher2
```

インストールせず使う場合:

```bash
bash ~/robotbase_ws_renya/bash/start_launcher2.sh
```

起動スクリプトは自分の場所からWSを特定します。別ユーザー名でも使えます。WSを移動した場合はショートカットのインストールを再実行してください。

**元の `koko_*` コマンドを維持するため、renya側の `bash_alias1.sh` / `bash_alias2.sh` を `.bashrc` に追加しないでください。** ランチャーの子端末だけがこれらを読み込みます。

## 分離される設定

| 項目 | ランチャー2 |
| --- | --- |
| 表示名 | ココちゃんランチャー2 |
| ショートカット | `koko_launcher2.desktop` |
| 専用コマンド | `~/.local/bin/koko_launcher2` |
| 設定・地図・Nav2パラメータ | cloneした `robotbase_ws_renya` 内 |
| PID・ログ | `/tmp/koko_launcher2_*` |
| 使用中の実機地図の状態 | `~/.koko_launcher2_nav2_current_map.yaml` |
| Gazebo partition | `koko_renya`（sim起動時は接尾辞付き） |
| ROS Domain | 実機54 / sim67 |

設定の正本は `robot.env` です。実機・simのROS Domainは既存値を維持しています。
導入先の元WSとsimのDomainが同じ場合は、ランチャー2のsim Domainを別の値に変更してください。「シミュレーション一括停止」はそのDomainまたはGazebo partitionに属するプロセスを対象にします。
Foxgloveは既存値8766なので、両WSのBridgeを同時起動する場合はrenya側の `ROBOTBASE_FOXGLOVE_PORT` を空いているポートへ変更してください。
同じ実機を操作するノードは両WSから同時起動しないでください。

元WSのデスクトップファイル、コマンド、設定ファイルはインストーラの書き込み対象に含まれません。
