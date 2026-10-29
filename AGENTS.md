# このワークスペースでの開発

- ユーザーの変更を保護する。git reset --hard、git clean、無断stash、既存変更の上書きは禁止。
- レストラン自律開発は独立worktreeのみ。元workspaceからRalphを起動しない。Loopの起動は実行前報告後の明示指示まで行わない。
- 実機用設定、robot.env、実機ドライバに触れず、シミュレーションのROS/Gazebo通信領域を分離する。
- restaurant_simの既存図面・モデルを活用し、寸法の推定と確認済み事項を区別する。レイアウト変更は根拠と通路幅の影響を記録する。
- 人の位置・姿勢・行動状態を同じシミュレーション時刻で検証する。テスト未実施を成功と書かない。
- 原本をsource/buildせずworktree固有のbuild/install/logを使用する。終了させるのは自分が起動したPIDのみ。
- 自動push/merge/commit、権限チェックの無効化は禁止。変更差分と試験証拠を残してレビュー可能にする。
- Ralph固有の優先順位と受入基準はRALPH_PROMPT.mdとtools/restaurant_ralph/EVALUATION.mdを参照する。
