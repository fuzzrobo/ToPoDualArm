# ToPoDualArm

C++ SDKは実機1台のDYNAMIXEL通信と関節・床・首の保護を担当します。Pythonからは `topoarm_sdk.Device`、ROS 2からは `ros2_control` が同じSDKを使います。同じ実機のポートはどちらか一方だけで開いてください。

### Pythonで実機に直結

```python
from topoarm_sdk import Device

with Device("config/robots/direct.yaml") as arm:
    arm.enable()                       # 接続とは別に駆動を許可
    q = arm.read().position            # 各指令の直前に読む（絶対角 rad）
    applied = arm.command(q)           # 補正後に受理された目標角
```

1台につき1プロファイルです。[個体設定](config/hardware/follower.yaml)は未校正のため、そのままでは実機に接続しません。`role: leader` は読み取り専用。指令解除は非常停止ではありません。

### macOSのU2D2受信遅延

個体設定の `connection.latency_timer` はミリ秒単位です。Macで2 msを使う場合は、この値を `2` に設定します。SDKがポートを開くたびにmacOSへ `IOSSDATALAT` で受信遅延を要求するため、起動前の手動設定は不要です。ポートの親ドライバが `AppleUSBFTDI` の場合は実測に基づき引数2を、それ以外はmacOS標準単位の2000 µsを渡します。ドライバが要求を拒否すると標準エラーに理由が出ます。要求が通ってもドライバによって実際の転送間隔は異なるため、18軸の読み取り時間で確認してください。SDKの応答期限の計算にもこの設定を使います。GUIの制御周期は別設定です。

### テレオペとBAMの読取プロファイル

フォロワーの個体設定で `options.read_profile` に `teleop`（既定）または `bam` を指定します。接続時に選択でき、サーボの動作モードと指令の可動域制限は共通です。

- `teleop` は通常周期に電流・速度・位置（126〜135番地）を全軸一括で読みます。起動時と約250 msごとに全診断を読み、電圧・温度・Torque状態は直近の診断値を保持します。各応答のAlertはその場で詳細確認します。リーダーとTorque OFF診断は各周期にTorqueを確認します。
- `bam` は毎周期64〜146番地を読みます。実機の3画面GUIに `--read-profile bam` を追加すると、その試行のLeRobotデータに速度、電流、PWM、電圧、温度、Torque、異常状態、サーボ時刻、目標角と位置・速度の内部軌道を `observation.bam.*` として保存します。通常収録は3画像と実測角・指令角です。
- Pythonで一時的に切り替えるには `Device(path, read_profile="bam")` を使います。`read()` と同じ周期に得た全関節の値は `last_states` で参照できます。短い読み取り周期では未取得のPWM・時刻・目標角・内部軌道はNaNです。

Macの3画面Quest GUI（`dev/viewer/record_quest.py`）では実機の制御・収録を100 Hz、画面更新を20 fpsに設定しています。カメラは30 fpsで、それぞれの制御サンプルに直近の同期画像を対応付けます。同じ画像が複数サンプルに入ります。実機の収録列 `observation.host_time_s` にはMacの単調時計で実測取得・指令受理・左／正面／右カメラ受信の5時刻（秒）が入ります。実効Hzや遅延の解析にはこの列を使います。MuJoCoでは制御・収録を20 Hz、3画面と全体ビューを独立した20 fps設定で動かします。収録した画像・実測角は同じ指令適用前の姿勢に対応します。

両モードで、ヘッドセットから見たコントローラーの位置・姿勢を `config/teleop/quest.yaml` の固定変換でEE目標にします。Gripで追従、Triggerで開閉、Aで収録開始・採用、Bで棄却します。左手・正面・右手の順に表示します。

MacではHomebrewの `eigen@3`、`yaml-cpp`、`tinyxml2` とTkが使えるPythonを用意し、`uv sync --locked --extra record` を実行します。`config/yubi.json` を `config/yubi-local.json` にコピーしてQuestのIPを設定してから、次でMuJoCoビューアを起動します。

```sh
uv run --extra record python dev/viewer/record_quest.py --config config/yubi-local.json
```

実機では校正済みのQuest設定を指定し、`--hardware` に直結プロファイル、`--cameras` に `left`、`front`、`right` をカメラのAVFoundation IDに対応付けたYAMLを渡します。macOSで実カメラを使う場合は `uv pip install pyobjc-framework-AVFoundation pyobjc-framework-CoreMedia` を実行します。`--position-first` を付けるとMuJoCoと同じ位置優先IKを使い、QuestとIKの更新を最大20 Hzにします。

### LeRobotで収録・再生・ACT推論

Ubuntuで `build-essential cmake libeigen3-dev libyaml-cpp-dev libtinyxml2-dev libegl1` と [uv](https://docs.astral.sh/uv/) を用意し、リポジトリ直下で実行します。

```sh
uv sync --locked --extra record --extra act --python 3.12
uv run --extra record python examples/episode.py record data/demo
uv run --extra record python examples/episode.py replay data/demo
bash examples/train_act.sh data/demo data/act 4
uv run --extra record --extra act python examples/act_sim.py data/act/checkpoints/last/pretrained_model
```

既定は模擬入力 → MuJoCo（18軸・3視点、20 Hz）。操作元は `--input meta --config 設定.json` または `--input leader --leader リーダー.yaml`。Quest＋Yubiは `--input quest`、グローブのみは `--input glove`（[設定例](config/yubi.json)）。実機収録は `--robot config/robots/direct.yaml --allow-hardware --cameras カメラ.yaml` と実入力を指定します。カメラ設定は front/left/right と各V4L2パスです。再生と付属のACT推論例はMuJoCo専用です。

### ROS 2 Humbleでリーダー・フォロワー

ROS環境で `rosdep install --from-paths ros2 --ignore-src -r -y` → `colcon build --base-paths ros2` → `source install/setup.bash` → `ros2 launch topoarm_bringup leader_follower.launch.py`。既定は両側mock。ROSノードはトピックで指令し、LeRobotやPythonバインディングは使いません。

C++ SDK単体は `cmake -S sdk -B build/sdk && cmake --build build/sdk`。[第三者コードの帰属](sdk/third_party/lib_dynamixel/README.md)
