# ToPoDualArm

ToPoDualArm 300 mmの実機を操作するSDKです。C++ SDKは関節I/Oと、関節可動域・床・首の指令制限を担当します。U2D2ではDynamixelの起動設定と異常復旧を行い、V-Sidoでは通常モードの状態取得と位置指令をUDPで行います。Pythonの `topoarm_sdk.Device` とROS 2の `ros2_control` は同じU2D2実装を使います。

## 設定

- `config/hardware/leader.yaml` と `config/hardware/follower.yaml` は標準16軸です。パンチルトを搭載した機体では `modules: [pan_tilt]` を指定して18軸にします。通信は2 Mbps、Return Delay Timeはレジスタ値 `10`（20 µs）が既定です。
- サーボのPID・速度プロファイルは機体YAMLの `actuator_profiles`、床の高さと余裕は `config/floor.yaml` にあります。モデルとパンチルト部品の編集手順は [300 mmモデル](models/300mm/README.md) を参照してください。
- 実機では機体別YAMLにポートと校正値を設定し、確認後に `hardware_verified: true` を指定します。Pythonでは `Device` に機体別YAMLを渡し、`role` で `leader` または `follower` を指定します。

`options.startup_configuration: apply` では、全IDと型番を確認し、動作モード、Drive Mode、Return Delay Time、PID、プロファイルの差分を読み戻しながら適用します。EEPROMの変更はトルクOFFで行います。フォロワーは駆動開始時に現在位置を目標にしてトルクをONにします。

`connection.latency_timer` の単位はミリ秒です。MacではSDKがポートを開く際に `IOSSDATALAT` で受信遅延を要求します。`AppleUSBFTDI` では実測に基づきミリ秒値を渡し、それ以外のドライバではマイクロ秒に換算します。

読み取りは `teleop` が既定です。通常周期に位置・速度・電流を一括取得し、温度や電圧は診断周期に取得します。詳細な診断値が毎周期必要なら `Device(path, role="follower", read_profile="bam")` を使用します。対応するHardware Errorの自動復旧は駆動中のフォロワーで動作し、リーダーは監視します。

## Python

[デモの環境構築](demo/README.md#環境構築)に従い、`demo/` で `uv sync` を実行します。ローカルSDKのPythonバインディングもビルドされ、`demo/.venv` に入ります。

```python
from topoarm_sdk import Device

with Device("path/to/follower.yaml", role="follower") as arm:
    arm.enable()
    q = arm.read().position  # 関節名ごとの角度（rad）
    applied = arm.command(q)
```

## ROS 2

Docker ComposeでHumbleまたはJazzyを選んで、双腕MoveItデモを起動できます。RVizの操作とUbuntuでの環境構築は [ROS 2の手順](ros2/README.md) を参照してください。

```sh
ROS_DISTRO=jazzy docker compose -f ros2/docker-compose.yaml up --build
```

関節I/Oのみ使う場合は `ros2 launch topoarm_bringup hardware.launch.py use_fake_hardware:=true` で起動します。関節状態は `/joint_states` で配信されます。位置や軌道は `/joint_trajectory_controller/follow_joint_trajectory` に関節名付きで指定します。左右腕・左右グリッパー・パンチルトの任意の関節を指定できます。

実機起動の `hardware.launch.py` は `backend:=u2d2` が既定です。V-Sido通常モードでも同じ `config/hardware/follower.yaml` の関節表と可動域を使います。

```sh
ros2 launch topoarm_bringup hardware.launch.py backend:=vsido config_file:=/path/to/follower.yaml vsido_peer_address:=192.168.4.40 hardware_verified:=true torque:=true
```

機体のIPを `vsido_peer_address` に指定します。受信するインターフェースは `vsido_bind_address` で指定できます。UDPポート、タイムアウト、CSVの送信順は機体YAMLの `vsido` にあります。受信角はパケット内のIDで照合します。個体の校正と可動域の確認後、個体設定で `hardware_verified: true` を指定します。

V-Sidoの既存の `command_order` は18軸です。16軸で位置指令を使う場合は機器側の受け付ける軸数と送信順を確認して設定します。`torque:=true` はROSからの位置指令送信を許可します。バイラテラルモードで状態だけ読む場合は `read_only:=true torque:=false` を指定します。UDP状態が既定の100 msを超えて途絶すると位置指令の送信が停止します。運転を再開するにはハードウェアの再構成が必要です。通信断後のサーボの挙動はV-Sido機器側に依存します。

## C++のビルド

```sh
cmake -S sdk -B sdk/build/cpp
cmake --build sdk/build/cpp
```

PythonバインディングのCMakeキャッシュは `sdk/build/python/` に生成されます。

[収録・ACT学習・推論デモ](demo/README.md)は `demo/` の環境で実行できます。

[第三者コードの帰属](sdk/third_party/lib_dynamixel/README.md)
