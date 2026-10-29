# 図面から作成したレストラン環境

ROS 2 Jazzy / Gazebo Harmonic 向けの簡易環境。元画像: `/home/ren/Downloads/Untitled.jpeg`。寸法測量や現地確認は行っていません。

## 起動

ワークスペースのルートで実行します。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select restaurant_sim --symlink-install
source install/setup.bash
ros2 launch restaurant_sim restaurant.launch.py scenario:=base
# 上のシミュレータを終了してから、次のいずれかを起動
ros2 launch restaurant_sim restaurant.launch.py scenario:=low
ros2 launch restaurant_sim restaurant.launch.py scenario:=medium
ros2 launch restaurant_sim restaurant.launch.py scenario:=high
```

`base` は固定家具のみ。low/medium/high は5/10/18人。Gazebo GUIが使えない環境では `gz sim -s -r <world.sdf>` でサーバーのみ起動できます。その場合、別端末でclockブリッジとpeopleノードを起動します。

```bash
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
ros2 run restaurant_sim people --ros-args -p use_sim_time:=true -p scenario:=low
```

## 読み取りと仮定

座標原点は図面左下、右が+x、上が+y。単位m。外形は17.0×11.5の矩形と仮定。印字された外寸は確実には判読できず、寸法値の転記ではありません。壁厚0.16、高さ2.5、テーブル高0.75、椅子高0.45を仮定しています。

- 左上: トイレと付帯室。内部は立入り対象外のブロックに簡略化。
- 左下: 厨房。内部設備はまとめた箱、客席との境界は直線・斜めのカウンターで表現。厨房内部の作業員動線は評価対象外。
- 上辺x=5.0〜6.6: 入口と仮定。図面の入口前張出し、扉の開閉、左側のサービス入口は省略。
- 会計: (5.25,9.7)が利用者停止位置。入口左の設備を会計と仮定。用途表示の確認はできていません。
- 給水: (8.5,1.3)が停止位置。下側設備帯を給水と仮定。図面上の給水器を特定したものではありません。
- 上辺6組、中央上部4組、中央菱形5組、右辺5組、下側共用テーブル1組と厨房沿いのカウンター椅子を配置。椅子数・各席の細部は簡略化しています。

各テーブルの中心・寸法・角度は `config/layout.json` の `tables`、全壁・家具は `obstacles`、人の着席停止位置は `seats`、設備利用位置は `goals` が正本です。`north_*`は上辺、`island_*`は上側の島、`diamond_*`は中央、`east_*`は右辺、`communal`は下側共用席です。

主な通路は厨房右側x≈7〜8、上側客席間y≈9.4、中央テーブル間、下側共用席の周囲。人用に障害物を0.29 m膨張した0.15 mグリッドで到達性を検査しています。すべての隙間がロボットに十分な幅とは限りません。

## 人間行動

各人は時間差で入店し指定位置へ移動、滞在、給水または短い散歩、席に戻り、会計、退出します。半数は初期着席相当として指定時刻に席へ出現します。乱数seed=42で速度・滞在時間を変化させます。lowは1人だけ給水、medium/highは約3分の1が給水。会計は全員が順番に目標とし、高混雑では集中が発生し得ますが時間帯や人数は設定変更で調整してください。C/Dの同行同期や空席選択は未実装です。

人は半径0.23 mの円柱（visual/collision両方）で、着席は椅子横の静止として表現します。椅子に身体を乗せるアニメーションはありません。A*の4近傍経路なので曲がり角は角ばります。他人から0.62 m未満への接近を停止で回避します。1秒以上接近停止した場合、他人を障害物として再経路探索します。退避場所のない狭路や占有中の目的地では待機を続けます。経路や状態遷移をROSログへ出力し、位置を `/restaurant/people` (PoseArray, world)へ配信します。

## ロボット実験

ロボットモデル・Nav2はこのパッケージでは起動しません。既存ロボットを同じworldへ配置し、自由空間の初期位置を選んでください。`/restaurant/robot_odom` にレストランworldと一致するmap/world座標のOdometryを配信すると人が接近を停止します。異なる座標系は事前にTF変換してください。閾値0.62 mは人とロボット中心間の仮値で、大型ロボットでは変更が必要です。

人の移動はGazeboのset_poseサービスによる位置更新であり、接触力に従う歩行ではありません。人からの接近は抑制しますが、ロボット側が人へ突進する場合の衝突回避は保証できません。Nav2の動的障害物処理と停止制御を併用してください。位置更新は別スレッドからset_pose_vectorで一括送信します。CLI起動・通信による表示遅延は残ります。time_scaleは待ち時間・出現時刻だけを加速し歩行速度は変えません。

LiDAR、接触、最小離隔、通過時間、到着成功率は実際のロボット構成で別途検証が必要です。現段階は近似レイアウトと行動の試作であり、実店舗の安全性の検証済みモデルではありません。

## ファイル

- worlds/: 固定家具版と3混雑版のSDF。モデルはSDFに内包し外部メッシュ不要。
- config/layout.json: 寸法・配置・目標位置。
- config/low.json, medium.json, high.json: 人の初期状態・行動設定。
- restaurant_sim/controller.py: 経路計算・状態遷移・Gazebo反映。
- launch/restaurant.launch.py: Gazebo、clock、人の起動。

## 実施した確認

- colcon build 成功。
- 4ワールドの `gz sdf -k` 成功。
- 全18停止位置から全サービス目標への経路テスト成功（pytest 1件）。
- Gazeboサーバー起動、set_poseによる人モデル移動要求に `data: true` を確認。
- 結合試験では当初200 msのサービス待機がタイムアウト。2000 msへ変更。変更後の全シナリオ完走、GUIの自然な移動、複数人の渋滞解消は未確認。
