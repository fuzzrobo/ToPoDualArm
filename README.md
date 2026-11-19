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
