# ToPoDualArm ROS 2

![RVizで両腕の軌道を実行したToPoDualArm](media/moveit-both-arms.png)

## 環境構築

Ubuntuでは、インストールしたROS 2 HumbleまたはJazzyの環境を読み込んでから、リポジトリ直下で実行します。rosdepの初期化は未設定の環境で一度だけ行います。

```sh
source /opt/ros/jazzy/setup.bash  # Humbleでは jazzy を humble に変更
sudo apt update
sudo apt install build-essential cmake python3-rosdep python3-colcon-common-extensions
sudo rosdep init  # 初回のみ
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths ros2 --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --base-paths ros2
source install/setup.bash
```

Macでは、リポジトリ直下からDocker Composeで起動します。Humbleを使う場合は `ROS_DISTRO=humble` を指定します。rosdepが各 `package.xml` を読んで依存をインストールします。

```sh
ROS_DISTRO=jazzy docker compose -f ros2/docker-compose.yaml up --build
```

Composeの既定はfakeで、RVizを表示せず起動します。MacのXサーバーがDockerからの接続を受け付ける設定なら、次のコマンドでRVizも開きます。

```sh
START_RVIZ=true DISPLAY=host.docker.internal:0 docker compose -f ros2/docker-compose.yaml up --build
```

## MoveItデモ

GUIを表示できるROS 2環境で起動します。既定はfakeで、RVizが開きます。

```sh
ros2 launch topoarm_moveit moveit.launch.py
```

標準は両腕・両グリッパーの16軸です。パンチルトと正面RealSense付きの18軸では、`config/hardware/follower.yaml` と同じディレクトリに個体YAMLを置き、`modules: [pan_tilt]` を指定して起動します。

```sh
ros2 launch topoarm_moveit moveit.launch.py config_file:=config/hardware/follower_pan_tilt.yaml
```

`/joint_states` には、駆動関節に加えて左右グリッパーのmimic関節も配信されます。

RVizの **MotionPlanning → Planning** で操作します。

1. **片腕**　**Planning Group** を `left_arm` または `right_arm` にし、手先のマーカーを動かして **Plan & Execute** を押します。
2. **両腕**　**Planning Group** を `both_arms`、**Goal State** を `reach` にして **Plan & Execute** を押します。
3. **グリッパー**　`left_gripper` または `right_gripper` を選び、**Goal State** を `open` または `closed` にして実行します。

![RVizでboth_armsのreachを選び、Plan & Executeで両腕を動かす](media/moveit-both-arms.gif)

参考にした構成は [TurtleBot3 LimeのMoveItパッケージ](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime/tree/humble-devel/turtlebot3_lime_moveit_config) です。

## 関節I/O

MoveItを使わずに関節状態と軌道指令を確認する場合は、`hardware.launch.py` を起動します。`/joint_states` に関節状態が配信され、`/joint_trajectory_controller/follow_joint_trajectory` に関節名と軌道を指定できます。

```sh
ros2 launch topoarm_bringup hardware.launch.py use_fake_hardware:=true
```

## 実機

校正と可動域を確認したフォロワーYAMLに `hardware_verified: true` を設定してから起動します。V-Sidoで接続する場合は `backend:=vsido vsido_peer_address:=<機器IP>` を加えます。

```sh
ros2 launch topoarm_moveit moveit.launch.py use_fake_hardware:=false \
  config_file:=/path/to/follower.yaml hardware_verified:=true torque:=true
```

標準の接続方式は `backend:=u2d2` です。[フォロワーYAML](../config/hardware/follower.yaml) の `actuator_profiles` にPIDと速度プロファイル、[床設定](../config/floor.yaml) に床の高さと余裕があります。機体の軸構成は [300 mmモデル](../models/300mm/README.md) を参照してください。

### V-Sido

V-SidoのUDPポート、タイムアウト、CSVの送信順は機体YAMLの `vsido` にあります。受信角はパケット内のIDで照合します。16軸で位置指令を送る場合は、機器側の軸数と送信順に `command_order` を合わせます。受信インターフェースは `vsido_bind_address` で指定できます。

バイラテラルモードで状態だけを読む場合は、`hardware.launch.py` に `read_only:=true torque:=false` を指定します。位置指令を送信中にUDP状態が既定の100 msを超えて途絶すると、指令送信が停止します。運転再開にはハードウェアの再構成が必要です。
