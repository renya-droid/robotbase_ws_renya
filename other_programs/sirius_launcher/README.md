# ココちゃんランチャー2

`robotbase_ws_renya` 専用のランチャーです。既存の `robotbase_ws` のランチャーと併用する手順は [別PCでの導入](../../DOCS/LAUNCHER2.md) を参照してください。

## 起動

```bash
bash ~/robotbase_ws_renya/bash/start_launcher2.sh
```

ショートカットと専用コマンドをインストールします。`.bashrc` は変更しません。

```bash
bash ~/robotbase_ws_renya/bash/install_launcher_shortcut.sh
~/.local/bin/koko_launcher2
```

## 設定

表示名と通信分離設定の正本は `~/robotbase_ws_renya/robot.env` です。

```bash
ROBOTBASE_DISPLAY_NAME="ココちゃん"
ROBOTBASE_ID="koko"
ROBOTBASE_LAUNCHER_NAME="ココちゃんランチャー2"
ROBOTBASE_LAUNCHER_ID="koko_launcher2"
ROBOTBASE_ROS_DOMAIN_ID="54"
ROBOTBASE_SIM_ROS_DOMAIN_ID="67"
ROBOTBASE_GZ_PARTITION="koko_renya"
ROBOTBASE_TF_PREFIX="robot"
```

表示名を変更した後はランチャーを再起動し、デスクトップショートカットのインストーラをもう一度実行してください。`ROBOTBASE_ID` はエイリアス名に関係するため、通常は変更しません。

ランチャーは `robotbase_ws_renya/bash/bash_alias2.sh` だけを読みます。`sirius_jazzy_ws` へフォールバックしないため、両WSが同じPCにあってもボタンが入れ替わりません。

## 通信とプロセスの分離

- 実機の子プロセス: `ROS_DOMAIN_ID=54`
- シミュレーションの子プロセス: `ROS_DOMAIN_ID=67`
- Gazebo Transport: `GZ_PARTITION=koko_renya`
- TF prefix: `ROBOTBASE_TF_PREFIX=robot`
- PID/ログ: `/tmp/koko_launcher2_*`
- Terminatorタブ: `[ココちゃんランチャー2] koko_*`

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
cd ~/robotbase_ws_renya
QT_QPA_PLATFORM=offscreen \
PYTHONPATH=other_programs/sirius_launcher \
pytest -q other_programs/sirius_launcher/tests
```

## 進入禁止領域（Keepout）

ランチャー上部の「進入禁止領域を編集」を押し、Nav2で使う地図YAMLを開きます。
地図上の対角２点で長方形、または各頂点をクリックして右クリックで多角形を確定します。
ペン・消しゴムはドラッグで塗り消しでき、太さをピクセル単位で指定できます。
ホイールで拡大縮小でき、「取り消し」で描画途中の点または最後の領域を削除できます。
形を修正する場合は取り消して描き直してください。

「この地図で Keepout を有効にする」を選んで保存すると、次回その地図でNav2を起動した際に
マスク配信とglobal/localコストマップのフィルターが自動で起動します。
無効にするにはチェックを外して保存し、Nav2を再起動します。
sim/realは起動時のuse_sim_timeに従います。既存地図を使うモードが対象で、SLAM中は適用しません。
Keepout有効中の地図切り替えスクリプトは変更を止めるため、新しい地図でNav2を起動し直してください。

マスクは元地図の隣の `.keepout/<地図名>/` に保存します。
`mask.png`（白=通行可能、黒=禁止）、`mask.yaml`、再編集と有効状態用の`settings.json`の３ファイルです。
地図本体は変更しません。地図の画像サイズ・原点・解像度が変わった場合は不一致を通知します。
地図名と保存場所で関連付けるため、地図を移動した場合は編集ツールで作成し直してください。

必要なPythonパッケージはPySide6、PyYAML、Pillowです。

単体版は `other_programs/keepout-editor/run.py` でも起動できます。

## エレベーターの地図別座標

ランチャー2の「リアル実験」→ `koko_elevator_map_profiles` で座標入力画面を開きます。
出発側の地図と到着側の地図をそれぞれ選び、ドアの左右端、かご頂点の X/Y、
かご・ドア前の local 範囲を入力します。数値は地図の `map` 座標系で、かご頂点は周回順です。

1. `current` または既存の環境設定を選び「新しい地図用に複製」を押します。
2. 新しい名前の環境で座標を入力し、実測値を確認してからチェック欄を選び、保存します。
3. 設定チェックの結果を確認し、エレベーター操作 GUI の「環境設定」でその名前を選びます。
   操作 GUI が開いたままなら、環境一覧を更新するため再起動します。
4. 元の地図に戻るときは、操作 GUI の「環境設定」を `current` または元の環境名に戻します。

設定は `~/kuwa_ws2/config/profiles/<名前>/` に保存されます。`current` は読み取り専用です。
複製した設定では目的階の人による確認が必要となり、古い waypoint 経路の運用メモも空欄になります。
地図切替方式や走行経路は複製元の設定と別に確認してください。制御ノードの起動中は保存できません。
RViz の形状プレビューは、使用中の地図に一致する確認済み設定が1つだけある場合に表示します。
同じ地図に複数の確認済み設定がある場合は、座標の取り違えを避けるため表示しません。

`1124ele3F_to_1124ele` の経路は、出発地図 `1124ele3F.yaml` と
出発ウェイポイント `maps_waypoints/waypoints/1124ele3F.yaml` を使います。
操作 GUI で同名の環境設定の制御ノードを起動してから、ウェイポイント走行で
`1124ele3F.yaml` を選択します。ドア外の1番地点でエレベーターサイクルを要求し、
退出・停止と地図切替が成功した場合だけ到着側 `1124ele.yaml` の経路へ引き継ぎます。
ウェイポイントとドア・かご座標はPGMからの推定値です。
位置、向き、車体余裕、地図と現物の一致を現地で確認してください。
確認用画像は `DOCS/elevator_1124_profile_review.png` です。
