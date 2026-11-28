# RVizユーザー設定

`koko_rviz_sim` と `koko_rviz_real` は、TF接頭辞ごとの `robotbase_<prefix>.rviz` をこのフォルダから読み込む。

- 現在の既定ファイル: `robotbase_robot.rviz`
- RVizの通常の `File -> Save Config` で同じファイルへ保存され、次回起動にも反映される。
- `robot.env` でTF接頭辞を変更すると、その接頭辞用ファイルがテンプレートから自動生成される。
- テンプレートの状態へ戻す場合はlaunchへ `reset_config:=true` を一度だけ渡す。現在のユーザー設定は上書きされる。

配布用テンプレートは `src/robotbase_bringup/rviz/robotbase.rviz` にあり、通常のSave Configでは変更されない。

エレベーター操作GUIで選んだ環境の、現在の地図に対応するドア（オレンジ）とかご外周（水色）を `Elevator map preview` に表示する。出発地図と到着地図の両方に対応する。制御ノードの起動中は、そのノードが配信する `Elevator door and cabin` を使用する。

`AMCL robot position` は `/amcl_pose` の自己位置と不確かさ、`Elevator exit target` は `/elevator_exit_preview` の退出候補を表示する。`arrival_exit_mode: map_only` では地図切替・初期位置設定までで停止し、退出候補は配信しない。

