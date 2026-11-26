# ToPoDualArm

ToPoDualArm 300 mmの実機を操作するSDKです。C++ SDKは1台分のDYNAMIXEL通信、起動設定、異常復旧、関節可動域・床・首の指令制限を担当します。Pythonの `topoarm_sdk.Device` とROS 2の `ros2_control` が同じSDKを使います。実機のシリアルポートは一方のプロセスで開きます。

## 設定

- `config/hardware/leader.yaml` と `config/hardware/follower.yaml` は18軸の初期設定です。通信の既定値は2 Mbps、Return Delay Timeはレジスタ値 `10`（20 µs）です。
- `config/models/servos.yaml` にサーボの共通設定、`config/floor.yaml` に床の高さと余裕を置きます。ロボット形状と関節構造の正本は `models/300mm/robot.urdf` と `models/300mm/meshes/` です。MJCFと軽量描画メッシュは同じモデルの `models/300mm/mujoco/` に置いています。
- 実機では機体別YAMLにポートと校正値を設定し、確認後に `hardware_verified: true` を指定します。Pythonでは `config/robots/` の `backend: direct` プロファイルを参考に、`hardware` で機体別YAMLを参照します。リーダーは `role: leader`、フォロワーは `role: follower` を指定します。

`options.startup_configuration: apply` では、全IDと型番を確認し、動作モード、Drive Mode、Return Delay Time、PID、プロファイルの差分を読み戻しながら適用します。EEPROMの変更はトルクOFFで行います。フォロワーは駆動開始時に現在位置を目標にしてトルクをONにします。

`connection.latency_timer` の単位はミリ秒です。MacではSDKがポートを開く際に `IOSSDATALAT` で受信遅延を要求します。`AppleUSBFTDI` では実測に基づきミリ秒値を渡し、それ以外のドライバではマイクロ秒に換算します。

読み取りは `teleop` が既定です。通常周期に位置・速度・電流を一括取得し、温度や電圧は診断周期に取得します。詳細な診断値が毎周期必要なら `Device(path, read_profile="bam")` を使用します。対応するHardware Errorの自動復旧は駆動中のフォロワーで動作し、リーダーは監視します。

## Python

Macでは `eigen@3`、`yaml-cpp`、`tinyxml2` を、Ubuntuでは `build-essential cmake libeigen3-dev libyaml-cpp-dev libtinyxml2-dev` を用意します。`uv sync --locked` でSDKをビルドできます。

```python
from topoarm_sdk import Device

with Device("path/to/follower-device.yaml") as arm:
    arm.enable()
    q = arm.read().position  # 関節名ごとの角度（rad）
    applied = arm.command(q)
```

## ROS 2

HumbleまたはJazzy環境で実行します。リーダー・フォロワーの起動は既定で両側mockです。

```sh
rosdep install --from-paths ros2 --ignore-src -r -y
colcon build --base-paths ros2
source install/setup.bash
ros2 launch topoarm_bringup leader_follower.launch.py
```

## C++のビルド

```sh
cmake -S sdk -B build/sdk
cmake --build build/sdk
```

[収録・ACT学習・推論デモ](python/demo/README.md)は `python/demo/` の環境で実行できます。

[第三者コードの帰属](sdk/third_party/lib_dynamixel/README.md)
