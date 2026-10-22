# ROS bag の記録と保存

`koko_record_rosbag` で開始します。保存先は `~/rosbag2_data/<入力した名前>/` です。

シミュレーションでは `koko_record_rosbag_sim` を使用します。
シミュレーション用ROS Domainへ切り替え、`--use-sim-time` と `/clock` の記録を有効にします。
記録・圧縮・終了処理と保存先は実機と共通です。
シミュレータを起動して `/clock` を配信してください。時計が来るまで記録は待機します。
存在しない実機用センサートピックは配信開始まで待機するため、simでも同じリストを使用できます。

エイリアスを再読み込みするには `source bash/bash_alias2.sh` を実行します。
直接起動する場合は `bash bash/startup_bash/record_rosbag.sh sim`、実機は末尾に `real` を指定します。

停止するときは Ctrl+C を押し、「保存が完了しました」と出るまで待ってください。
記録プロセスは端末とは別のセッションで起動し、スクリプトが SIGINT を送り、
索引の書き込みとプロセス終了を待ちます。保存中の追加の Ctrl+C は無視します。
SIGTERM・SIGHUP も同じ終了処理を行います。

`params/common/rosbag_mcap.yaml` で MCAP の索引を有効にし、内部チャンクを Zstd 圧縮します。
終了後は `tools/finalize_rosbag.py` で全 MCAP の Footer と Summary の有無を確認します。
これはファイル全体のCRCやメッセージ内容を検査する処理ではありません。

追従失敗の解析用に `/rosout`、`/cmd_vel_smoothed`、`/hokuyo_scan` と、
`/follow_path`・`/navigate_to_pose`・`/navigate_through_poses` の
`/_action/status`・`/_action/feedback` も記録します。
アクションの隠しトピックを記録するため `--include-hidden-topics` を指定しています。
`/optimal_trajectory` も記録対象ですが、現在のMPPI設定は `visualize: false` のため、
配信されていなければbagには入りません。記録対象への追加だけでは配信は有効になりません。

未完了のファイルは MCAP CLI で修復し、元ファイルを `*.mcap.damaged` に残します。
修復後は `ros2 bag reindex` で ROS bag のメタデータを更新します。
修復ツールがない場合や修復に失敗した場合は、成功と表示せず元ファイルを保持します。
強制終了（SIGKILL）・電源断では終了処理を実行できず、破損した末尾は失われる可能性があります。

修復ツールの探索順は `MCAP_BIN`、PATH の `mcap`、`.runtime/bin/mcap` です。
このワークスペースには公式 MCAP CLI v0.4.0（Linux amd64）を `.runtime/bin/mcap` に配置済みです。
`.runtime/` は Git 管理対象外のため、別の環境では [公式配布元](https://github.com/foxglove/mcap/releases/tag/releases%2Fmcap-cli%2Fv0.4.0)
から対象環境用の実行ファイルを配置してください。

回帰テスト（ROS Jazzy が必要）:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tools -p test_record_rosbag.py
```

SIGINT・SIGTERM・SIGHUP、保存中の追加シグナル、起動失敗、
末尾が切れたファイルの修復・元データ保持・メタデータ再構築を検証します。
