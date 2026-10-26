# ROS bag の記録と保存

`koko_record_rosbag` で開始します。保存先は `~/rosbag2_data/<入力した名前>/` です。

シミュレーションでは `koko_record_rosbag_sim` を使用します。
シミュレーション用ROS Domainへ切り替え、`--use-sim-time` と `/clock` の記録を有効にします。
記録・圧縮・終了処理と保存先は実機と共通です。
シミュレータを起動して `/clock` を配信してください。時計が来るまで記録は待機します。
存在しない実機用センサートピックは配信開始まで待機するため、simでも同じリストを使用できます。

エイリアスを読み込むにはワークスペースで次を実行します。スクリプトとYAMLの変更なのでcolconによる再ビルドは不要です。

```bash
cd ~/robotbase_ws_renya
source bash/bash_alias1.sh
source bash/bash_alias2.sh
koko_record_rosbag      # 実機：通常時刻
# または koko_record_rosbag_sim
```

従来どおりEnter、ファイル名、実験メモを入力します。保存先、既存名の確認、ランチャーからの起動は共通です。
すでに起動中のランチャーにはPythonの変更が反映されないため、記録を安全に終了した後でランチャーも再起動してください。
直接起動する場合は `bash bash/startup_bash/record_rosbag.sh sim`、実機は末尾に `real` を指定します。

停止するときは Ctrl+C を押し、「保存が完了しました」と出るまで待ってください。
記録プロセスは端末とは別のセッションで起動し、スクリプトが SIGINT を送り、
索引の書き込みとプロセス終了を待ちます。保存中の追加の Ctrl+C は無視します。
SIGTERM・SIGHUP も同じ終了処理を行います。
記録プロセスが自分で終了した場合も、保存確認・修復中の追加シグナルを無視します。
保存待ちに時間制限やSIGKILLへの自動昇格は設けていません。

ランチャーの記録ボタンの停止では、記録ラッパーだけへ停止を要求し、2秒後の強制終了や端末の自動クローズを行いません。
「sim一式終了」では記録ラッパーへ保存要求を送り、writer・修復・reindex・記録端末を強制終了対象から除外します。
sim終了後も保存が続く場合があるので、記録端末の結果を確認してください。
実機/simのDomainや環境切替はmainの既存activateスクリプトとrobot.envを使います。

結果は次のように区別します。

- MCAP作成前の非ゼロ終了：起動失敗。
- MCAP作成後の非ゼロ終了：記録プロセスの異常終了。保存確認の成否を別途表示し、非ゼロで終了。
- 検査・修復・reindexの失敗：保存確認失敗。保存完了とは表示しません。
- recorderの正常終了と保存確認の両方が成功：保存完了。

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
分割ファイルも含め、bagディレクトリ直下の全 `*.mcap` を検査します。
一時ファイル `*.mcap.recovering` を検証した後に置き換え、以前の `.damaged` や `.recovering` は上書きしません。
修復後は `ros2 bag reindex -s mcap` で ROS bag のメタデータを更新します。
途中失敗時には `.mcap-reindex-required` を残し、再実行時もreindex成功までは完了扱いにしません。
修復ツールがない場合や修復に失敗した場合は、成功と表示せず元ファイルを保持します。
強制終了（SIGKILL）・電源断では終了処理を実行できず、破損した末尾は失われる可能性があります。

停止済みのbagを手動で確認・修復する場合は次を使います。記録中のbagには実行しないでください。

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/finalize_rosbag.py ~/rosbag2_data/<bag名>
```

既存の修復途中ファイルがある場合は自動処理を止めます。元データと既存ファイルを確認・保管してから再試行してください。

修復ツールの探索順は `MCAP_BIN`、PATH の `mcap`、`.runtime/bin/mcap` です。
このワークスペースには公式 MCAP CLI v0.4.0（Linux amd64）を `.runtime/bin/mcap` に配置済みです。
`.runtime/` は Git 管理対象外のため、別の環境では [公式配布元](https://github.com/foxglove/mcap/releases/tag/releases%2Fmcap-cli%2Fv0.4.0)
から対象環境用の実行ファイルを配置してください。

必要なものはPython 3、ROS 2 Jazzyの `ros2 bag` / `rosbag2_py` / MCAP storage plugin、およびランチャー・sim終了処理の `python3-psutil` です。
通常の保存検査にはMCAP CLIは不要ですが、未完了ファイルの自動修復には必要です。

```bash
sudo apt install ros-jazzy-ros2bag ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap python3-psutil
# 公式配布元からOS/CPUに合うCLIをダウンロードした後：
mkdir -p .runtime/bin
# ダウンロードした実行ファイルを .runtime/bin/mcap に配置
chmod +x .runtime/bin/mcap
.runtime/bin/mcap --version
```

任意の場所へ配置する場合は `export MCAP_BIN=/absolute/path/to/mcap`、またはPATHへ登録します。
MCAP_BINが指定されたのに実行不能な場合は、別のCLIへ黙って切り替えず失敗として報告します。
このPCではLinux amd64版v0.4.0を公式配布のSHA256と照合しました。

回帰テスト（ROS Jazzy が必要）:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tools -p test_record_rosbag.py
```

SIGINT・SIGTERM・SIGHUP、保存中の追加シグナル、起動失敗、
末尾が切れたファイルの修復・元データ保持・メタデータ再構築を検証します。

## mainへの移植確認（2026-10-23）

main `c72dc89` と参照 `origin/renya/camera-nav` の `c2dd9aa`、作業ツリーを確認して移植しました。
前回のwaypoint移植や既存地図などの変更は保全し、作業前のtracked差分・全untrackedを `/tmp/robotbase-rosbag-backup-20261023/` に保存しました。
ブランチ全体のマージ、実機走行、commit、pushは行っていません。

記録処理・MCAP設定・修復ツール・関連テスト・記録用sim aliasを参照ブランチから移植しました。
追加で、自然終了後の保存確認中のシグナル保護、reindex再試行、保存待ちを打ち切るランチャー/sim一式終了の処理を修正しました。
mainの既存29トピックはすべて維持しています。`/optimal_trajectory` とSpinのstatus/feedbackは元から記録対象で、そのまま残しています。
Nav2・MPPI・速度設定や環境分離設定は今回変更していません。

テストは一時ディレクトリに新規MCAPを作り、テスト用コピーの末尾を切って実行しています。既存のbagは変更していません。
シグナル試験はDDS通信を行わないテスト用ros2フロントエンドと、本物のROS MCAP writerを使用します。
修復には公式MCAP CLI v0.4.0、reindexにはインストール済みROS CLIを使用し、修復後の読み出しも確認します。
MCAPのZstdチャンク、MessageIndex、ChunkIndexが生成されることも実ファイルで確認します。
実機のデータ配信・長時間記録・電源断からの復旧は未確認です。

| 確認 | 結果 |
|---|---|
| rosbag関連回帰（正常終了、3種シグナル、保存中の追加シグナル、起動失敗・異常終了・確認失敗、分割MCAP修復、原本保持、reindex失敗と再試行、圧縮・索引、実機/sim入力、ランチャー・sim一式終了） | 17件成功・skipなし |
| 既存ランチャーのタブ・エイリアス・Domain設定テスト | 4件成功 |
| Bash構文 / Pythonコンパイル / git diff --check | 成功 |
| 作業前からのtracked差分・untrackedファイルの保全比較 | 完全一致 |

ログは [verification/rosbag_port_20261023](verification/rosbag_port_20261023/) に保存しています。

## 端末切断で保存処理が中断する不具合の修正（2026-10-23）

15F-1023の記録について、Footer/Summaryがなく、ランチャーログにも停止・保存処理がないことを読み取りで確認しました。元のMCAPは変更していません。
調査では、端末やランチャーの表示用teeが先に閉じた場合、停止処理のechoでBashがSIGPIPE終了し、recorderへSIGINTを送る前に保存処理が消える経路を再現しました。修正前のテスト用ファイルも索引なしとなり、wrapperの終了コードは-13でした。以前のシグナル試験は出力先が開いたままで、この経路をカバーできていませんでした。

記録開始時に `~/rosbag2_data/.recording-logs/`（ROSBAG_DIR指定時はその直下）へ専用ログを作成します。画面出力は別セッションのrelayを介し、端末切断後もファイル出力を続けます。recorder・検査・修復の出力先を保持することでSIGPIPEやBrokenPipeErrorによる保存処理の中断を防ぎます。保存確認ログのパスは開始時に表示します。
ランチャーでも記録ラッパーを特定できないときに汎用の子プロセス終了へ進む処理を禁止しました。

修正後の確認結果：

- rosbag回帰20件成功。出力パイプ切断後のSIGHUP/SIGTERM、制御端末のクローズ、ランチャーのtee消失後の停止を追加。
- 既存ランチャー回帰4件成功。Bash構文・git diff --checkも成功。
- 本物のROS recorderを専用Domain 228・localhost・テスト用トピックで確認。出力切断後のSIGTERMと制御端末クローズの両方で正常終了、Footer/Summaryとmetadata.yamlを確認し、144件・146件のテストメッセージを読み出せた。保存先は/tmpで、実機driverは起動していない。

ログと再現スクリプトは [verification/rosbag_terminal_fix_20261023](verification/rosbag_terminal_fix_20261023/) に保存しています。
再ビルドは不要です。次の記録から新しいスクリプトが使われます。ランチャー側の追加修正も反映するため、次の記録前にランチャーを開き直してください。端末を閉じてしまった場合も、専用ログで保存確認の成否を確認できます。
