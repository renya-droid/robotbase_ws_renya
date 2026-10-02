# ココちゃん用Bashショートカット

シリウス用aliasとの競合を避けるため、このWSが公開するコマンドはすべて `koko_*` です。

`.bashrc`へ次を追加します。この開発PCには設定済みです。

```bash
source ~/robotbase_ws_renya/bash/bash_alias1.sh
source ~/robotbase_ws_renya/bash/bash_alias2.sh
```

読み込んだだけでは現在の `ROS_DOMAIN_ID` を変更しません。`koko_*` の実行時だけ `robot.env` に従ってココちゃん用DomainとGazebo partitionへ切り替わります。

`koko_env` はSirius由来の検索パスも除去します。`koko_build` もこのクリーン環境でビルドするため、`install/setup.bash` がSiriusを親overlayとして再登録することを防ぎます。

主なコマンド:

```bash
koko_src       # 実機: robotbase_ws_renya + ROS_DOMAIN_ID=54
koko_sim_src   # シミュレーション: robotbase_ws_renya + ROS_DOMAIN_ID=67
koko_build
koko_sim
koko_rviz_sim
koko_slamtoolbox_sim
koko_nav2_sim_map
koko_nav2_sim_slam
koko_keyop2_sim
koko_map_save_sim
koko_hokuyo
koko_slamtoolbox_real
koko_nav2_real
koko_nav2_real_slam
koko_rviz_real
koko_launcher
```

表示名・Domain IDの変更とデスクトップUIについては `DOCS/LAUNCHER.md` を参照してください。
