# レストラン・配膳ロボット研究用自律開発

## 使命とスコープ
ROS 2 Jazzy / Gazebo Harmonicの既存restaurant_simを、平面図に基づく人混在ナビゲーションの再現可能な実験基盤へ改善する。新規の無関係なデモに置き換えない。研究質問: 混雑度と人の離席・給水・会計行動が、配膳ロボットの到着率、所要時間、離隔、停止時間へ与える影響は何か。改善はこの問いへの測定可能性で判断する。

まずAGENTS.md、src/restaurant_sim/README.md、tools/restaurant_ralph/BASELINE.md、EVALUATION.md、PROGRESS.mdを読む。元ワークスペースでは絶対にLoopを実行しない。RALPH_WORKTREE.jsonとgit worktree listで隔離を確認。初回は現在のHEADと差分を記録し、以降は進捗と失敗ログから再開する。

## 保全と範囲
既存変更をreset/clean/stash/checkoutで捨てない。原本・他のworktree・実機・Git設定・既存起動プロセスを変更しない。push/merge/自動コミットは禁止。依存導入でsudoやネットワークが必要なら未達として報告。センサー実機ドライバを起動しない。ROS_DOMAIN_ID=89、GZ_PARTITION=restaurant_ralph_research、ROS_LOCALHOST_ONLY=1を使用し、競合時は中止。自分が起動したPIDだけを終了する。build/install/logはworktree内。元環境のinstallをsourceしない。

通常の変更対象はsrc/restaurant_sim、必要なら新規src/restaurant_interfaces、tools/restaurant_ralph。robotbase_simとexternal_camera_people_layerは既存実装を読み再利用、必要最小限の拡張と回帰テストを行う。params/real、実機launch、ベンダーNav2本体、robot.envは変更禁止。既存sim設定は専用設定を派生させて使用する。

## 段階的な実装
1. ベースラインを実行・記録する。2件の既存単体テストを維持。SDF・JSON・図面を照合する。会計と給水、外寸など既存の仮定を明記し、判読不能な寸法を確定値にしない。
2. layout.jsonを単一の正本とする再生成器を復元し、SDF、Nav2静的地図、座席ID、利用者停止位置、配膳到達位置を生成する。床と障害物を区別。椅子の上と通路停止位置を混同しない。図面ピクセル→world変換と誤差・仮定台帳を記録。自動経路確保のために家具を無断で消さない。参照画像はRALPH_WORKTREE.jsonのreference_imageで読む。
3. controllerを経路探索・行動状態機械・Gazebo通信・ROS配信に分離。秒数はsim clockを正本とし、一時停止・再開・時間巻戻り・過負荷に対処。通信失敗中に内部位置だけ先行して成功扱いしない。pose更新の遅延・実位置・状態の一致を検査。CLI連続起動の置換はインストール済みAPI/bridgeを調べて選ぶ。
4. ENTERING→WALKING_TO_SEAT→SEATED→STANDING→WALKING_TO_WATER→GETTING_WATER→RETURNING→SEATED→WALKING_TO_CASHIER→PAYING→EXITING→EXITEDを明示。給水不要/散歩/初期着席を有効な分岐とする。有限時間待機、席占有、給水・会計のキュー、退店と資源解放を実装。ペア同行と片方のみ給水を追加。全員を歩かせ続けることで停止問題を隠さない。
5. 着席中は静止し、立位/座位と表示が同期。円柱継続可、見た目は研究に必要な区別に留める。椅子への遷移で壁や机を横断しない。対向、交差、座席付近、会計列での停止を再現し、再経路・優先順位・退避を実装。通過不能なら診断とタイムアウトを出しテスト失敗にする。
6. /restaurant/agentsにstamp、frame_id、永続id、pose（zと向きを含む）、twist、posture(enum STANDING/SEATED)、activity、seat_id、goal_id、activeを含む型付きスナップショットを配信。message packageが必要ならament_cmakeのrestaurant_interfacesを追加。人物IDを配列indexにしない。PoseArray互換トピックは維持。退店・stale・空配列を定義。TFでmap/world/robot/odomを実際に変換し同名扱いしない。
7. robotbase_simのロボット・センサー・navigation.launch.pyと専用地図/paramsを使いレストラン統合launchを提供。単一Gazebo/clock/map→robot/odom→robot/base_linkの所有者を決める。headless、seed、scenario、GUI有無を引数化。external_camera_people_layerの全件PoseArray契約を尊重し専用adapterで接続可能にする。ground-truth人位置と疑似センサー入力を区別。local/global costmapと人の消去、ロボットfootprint、動的停止を検証。
8. EVALUATION.mdの独立評価器、シード付き統合試験、CSV/JSON/rosbag記録、集計、再実行コマンドを実装。評価器を先に負例で検証する。基準未達をテスト削除、skip、閾値緩和、人数削減、衝突無効化で隠さない。

## 反復と完了
各反復は最優先の未達項目を1つ選び、失敗再現→最小修正→関連テスト→PROGRESS.md更新を行う。ログはartifacts/restaurant/<run-id>/へ。途中で止まっても続行できるよう次の操作を具体的に残す。予算上限を達成とみなさない。同じ環境障害が3回続く場合は原因と必要手順を記録して <promise>RESTAURANT_BLOCKED</promise> を出す。
全完了はEVALUATION.md全gate成功、証跡manifestにHEAD/diff hash/設定/seed/コマンド/exit codeが揃い、独立評価器が成功し、READMEの再現手順が別ビルドで実行できる時だけ。表示だけ、単体テストだけ、自己申告だけで完了しない。最終レポートは図面との差、仮定、結果、既知制限、再現コマンドを日本語で作る。その条件を全て満たした場合のみ <promise>RESTAURANT_RESEARCH_READY</promise> を出す。未実行は未確認と書く。
