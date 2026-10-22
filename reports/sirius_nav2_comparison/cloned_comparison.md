# クローン版Siriusとの再比較（2026-10-22）

クローン版 `/home/ren/sirius_jazzy_ws`、commit `b444fb7` を読み取りました。
前の比較対象だった10月2日の展開版とは、設定・ToPoseのBTに変更があります。
ココちゃん側も現在の実機・simファイルを読み直しています。
設定変更・ノード再起動・クローン側への書き込みは行っていません。

## 前のSirius比較から変わった点

|項目|前の展開版|クローン版|
|---|---|---|
|コントローラ|FollowPath（MPPI）のみ|FollowPath + WaitPath（Regulated Pure Pursuit）|
|MPPI内部再試行上限|未指定、実装既定1|明示2|
|通常simの前進優先重み|15|10|
|通常simのCostCritic重み|16|10|
|通常simのnear_collision_cost|120|150|
|BT再読み込み|always_reload_bt_xml未指定|true|
|ToPoseのコントローラ切替|単一の選択コントローラ|Switch2でWaitPath/FollowPath/その他へ分岐|
|ToPose通常追従の失敗後|WouldAControllerRecoveryHelp判定後にlocalをクリア|エラー種別判定を挟まずlocalをクリアして再試行|
|ToPoseのWaitPath失敗後|処理なし|localクリア→2秒待機→再試行、上限1,000,000回|

sim_no_semanticも通常simと同じ重みへ変更されています。
sim_slamは前進優先15・CostCritic16・near_collision_cost120を維持しており、通常simとは異なります。
上記の旧パラメータ値は前回比較記録と、残っている10月2日版paramsでも照合しました。
旧ワークスペース全体は現在の元の場所にないため、旧BTの説明は前回取得した内容との比較です。

## WaitPathと姿勢合わせ

WaitPathは `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`。
`controller_selector` で選択されたときにToPoseのBTが切り替えます。
**MPPIが失敗すれば自動でWaitPathに切り替わる、という構成ではありません。**
クローンには選択を行うSirius側クライアントのサブモジュールが未展開のため、
そのクライアントの運用条件までは確認できません。

`WaitPath.use_rotate_to_heading = false` です。
`rotate_to_heading_min_angle = 0.785` などの数値はありますが、有効化フラグがfalseのため、
この設定で「大きな方位差なら最初に旋回する」とは言えません。
MPPI側にもRotation Shimはありません。

SiriusのToPose/ThroughPosesの既定BTには引き続きSpin約90度があり、
ココちゃんの起動BTでは省かれています。WaitPathの待機再試行ルートはSpinの一般復帰へ進む前に再試行する構造です。
ThroughPosesの既定BTにはToPoseのSwitch2/WaitPathの分岐は追加されていません。

## 今のココちゃんとの差

|項目|ココちゃん実機・sim|クローンSirius実機・通常sim|
|---|---|---|
|予測時間|2.8秒|6秒|
|参照経路距離|3m|6m|
|前進優先の重み|40|10|
|CostCritic重み|4|10|
|VelocityDeadbandCritic|あり、重み35|なし|
|平滑化のdeadband|前進0.1m/s未満をゼロ|全軸0|
|探索ノイズ再生成|false|true|
|後退下限|−0.3m/s|−0.6m/s|
|コントローラ|MPPIのみ|MPPI + 選択式RPP|
|MPPI内部再試行上限|未指定、既定1|2|
|Rotation Shim|なし|なし|
|BTのSpin復帰|省略|あり|

MPPIのC++ヘッダ・ソース63ファイルは、クローン版とココちゃん側で全て同一です。
相違はその実装へ渡すパラメータとBT・周辺構成にあります。
共有ライブラリや稼働中設定を同一と確認したものではありません。

全パラメータの最新差分: [cloned_parameter_differences.csv](cloned_parameter_differences.csv)。
クローン側は実機用paramsと通常sim/no_semanticを起動スクリプトが選択します。
goalで明示したBTや実行時上書きは別途確認が必要です。
