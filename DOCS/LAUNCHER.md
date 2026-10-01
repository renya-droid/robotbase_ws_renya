# ココちゃん用Bashとランチャー

## 目的

この開発PCにはSIRIUS用の `sirius_jazzy_ws` と新機体用の `robotbase_ws` が共存します。コマンド名、ROSグラフ、Gazebo Transport、ランチャーのPIDを分離し、誤って別機体を操作しない構成にしています。

## 設定の正本

`~/robotbase_ws/robot.env`:

```bash
ROBOTBASE_DISPLAY_NAME="ココちゃん"
ROBOTBASE_ID="koko"
ROBOTBASE_ROS_DOMAIN_ID="57"
ROBOTBASE_SIM_ROS_DOMAIN_ID="58"
ROBOTBASE_GZ_PARTITION="koko"
ROBOTBASE_TF_PREFIX="robot"
ROBOTBASE_FOXGLOVE_ADDRESS="0.0.0.0"
ROBOTBASE_FOXGLOVE_PORT="8766"
```

- `ROBOTBASE_DISPLAY_NAME`: UI、ターミナルタイトル、デスクトップ表示名
- `ROBOTBASE_ID`: `koko_*`や`/tmp/koko_launcher_*`の安定識別子
- `ROBOTBASE_ROS_DOMAIN_ID`: 実機用ROS 2 Domain
- `ROBOTBASE_SIM_ROS_DOMAIN_ID`: シミュレーション用ROS 2 Domain。実機と異なる値にする
- `ROBOTBASE_GZ_PARTITION`: Gazebo Transportの分離名
- `ROBOTBASE_TF_PREFIX`: TF接頭辞。Gazebo、driver、Nav2、SLAM、RVizへ反映
- `ROBOTBASE_FOXGLOVE_ADDRESS` / `ROBOTBASE_FOXGLOVE_PORT`: Foxgloveの待受先。Siriusの8765と分離

正式名称が決まったときは、通常 `ROBOTBASE_DISPLAY_NAME` だけを変更します。その後ランチャーを再起動し、次を実行してください。

```bash
~/robotbase_ws/bash/install_launcher_shortcut.sh
```

`ROBOTBASE_ID`を変更するとalias名も変更が必要になるため、表示名変更だけでは変更しません。

## Bashコマンド

`.bashrc`にはSIRIUSのaliasに加えて次を設定済みです。

```bash
source ~/robotbase_ws/bash/bash_alias1.sh
source ~/robotbase_ws/bash/bash_alias2.sh
```

すべて `koko_*` 接頭辞なので、SIRIUSの `nav2`、`roboteq`、`src`などと名前が衝突しません。

| コマンド | 用途 |
|---|---|
| `koko_src` | Domain 57を設定してrobotbase_wsをsource |
| `koko_sim_src` | Domain 58を設定してrobotbase_wsをsource |
| `koko_build` | robotbase_wsをビルド |
| `koko_sim` | Gazebo単体 |
| `koko_rviz_sim` | シミュレーション用RViz単体 |
| `koko_slamtoolbox_sim` | シミュレーション用SLAM単体 |
| `koko_nav2_sim_map` | 同梱・保存済み地図を一覧から選ぶシミュレーションNav2 |
| `koko_nav2_sim_slam` | 地図なしでSLAMとNav2を同時起動 |
| `koko_keyop2_sim` | シミュレーション用手動操作 |
| `koko_map_save_sim` | シミュレーション地図保存 |
| `koko_roboteq` | 新機体のRoboteq |
| `koko_hokuyo` | Hokuyo 2D LiDAR（`/hokuyo_scan`） |
| `koko_keyop2` | 新機体の手動操作 |
| `koko_slamtoolbox_real` | 実機SLAM単体 |
| `koko_nav2_real` | 新機体の実機Nav2 |
| `koko_nav2_real_slam` | 実機で地図を生成・更新しながらNav2を実行 |
| `koko_rviz_real` | 実機用RViz単体 |
| `koko_foxglove` | 実機DomainのFoxglove Bridge（8766） |
| `koko_foxglove_sim` | シミュレーションDomainのFoxglove Bridge（8766） |
| `koko_launcher` | GUIランチャー |

aliasファイルをsourceしただけでは、SIRIUS端末のDomain 56を変更しません。`koko_src`または`koko_env`で実機Domain 57、`koko_sim_src`または`koko_sim_env`でシミュレーションDomain 58になります。同じターミナルでSIRIUSへ戻る場合は新しいターミナルを開くのが安全です。

## UIランチャー

起動方法:

```bash
koko_launcher
```

またはデスクトップの「ココちゃん ランチャー」を開きます。

ランチャーは `robotbase_ws/bash/bash_alias2.sh` のみを読み、SIRIUSのaliasファイルへフォールバックしません。子ターミナルでは `.bashrc` 読み込み後に、ココちゃん用の次の値を必ず再設定します。

```text
ROS_DOMAIN_ID=57  # 実機。シミュレーションボタンでは58
GZ_PARTITION=koko
ROBOTBASE_TF_PREFIX=robot
```

`koko_env` は値を上書きするだけでなく、`AMENT_PREFIX_PATH`、`PYTHONPATH`、`GZ_SIM_RESOURCE_PATH`などから `sirius_jazzy_ws` の項目を除去します。したがってSiriusをsource済みのターミナルやランチャー子端末でも、ココちゃんのpackage・mesh・launchを優先できます。

プロセス管理も次のように分離されています。

```text
/tmp/koko_launcher_<alias>.pid
/tmp/koko_launcher_<alias>.log
Terminator tab: [ココちゃん] <alias>
```

UIは5タブです。Gazebo、RViz、シミュレーション地図作成、2種類のシミュレーションNav2は「シミュレーション」タブに集約し、すべてDomain 58で起動します。実機用 `twist_mux` と `koko_nav2_real_slam` は「リアル実験」タブです。実機用`koko_foxglove`とシミュレーション用`koko_foxglove_sim`は「ユーティリティ」タブにあります。外部連携タブ、LLM、ZED/SAM3関連ボタンはありません。

ランチャー上部の赤い「シミュレーション一式を終了」ボタンは、確認後に`ROBOTBASE_SIM_ROS_DOMAIN_ID`（既定58）またはココちゃん用Gazebo partitionに属するプロセスをまとめて終了する。SIGINTで正常終了を試し、残ったものだけSIGTERM、最終的にSIGKILLを使う。実機Domain 57とSIRIUSのプロセスは対象にしない。`/clock publisherが既にあります`と表示された場合や、Gazeboを最初から起動し直す場合に使用する。

上部の「シミュレーション ROS_DOMAIN_ID」は0〜232から選択でき、「Domainを保存」で`robot.env`の`ROBOTBASE_SIM_ROS_DOMAIN_ID`へ保存する。初期値は58で、次に起動するGazebo、シミュレーションRViz、Nav2、SLAM、Keyop、Waypoint、Foxgloveへ共通反映される。実機Domain 57と同じ値は保存できない。

古いDomainのプロセスを残したまま値だけ変えると、画面上は衝突を回避しても古いGazeboがバックグラウンドで動き続ける。このため、現在のシミュレーションDomainまたはGazebo partitionのプロセスが見つかった場合は保存を拒否する。赤い終了ボタンを先に押してから保存する。YAMLやPythonのビルドは不要で、保存後に新しく起動するプロセスから反映される。

UIを起動できない場合は、同じ処理を端末から直接実行できる。

```bash
bash ~/robotbase_ws/bash/startup_bash/stop_simulation.sh
```

`koko_rviz_sim` と `koko_rviz_real` は `rviz/robotbase_<TF接頭辞>.rviz` を読み込む。RVizの通常の `File -> Save Config` で保存した表示構成は、次回の起動でもそのまま使われる。

## デスクトップショートカット

次の2か所へ設置済みです。

```text
~/Desktop/koko_launcher.desktop
~/.local/share/applications/koko_launcher.desktop
```

再生成スクリプトは `bash/install_launcher_shortcut.sh` です。`robot.env` の表示名と現在のワークスペース絶対パスをテンプレートへ反映し、デスクトップ側には実行権限とtrusted metadataを設定します。

## 共存確認

新しいターミナルで:

```bash
echo "$ROS_DOMAIN_ID"   # SIRIUS既定: 56
alias nav2              # SIRIUS
alias koko_nav2_sim_map # ココちゃん・既存地図

koko_env
echo "$ROS_DOMAIN_ID"  # 57
echo "$GZ_PARTITION"   # koko
echo "$ROBOTBASE_TF_PREFIX" # robot
```

異なるROS Domain間ではトピック、サービス、アクションが相互に見えません。同じロボットを複数PCで操作する場合は、対象PC同士のDomain値を揃える必要があります。
