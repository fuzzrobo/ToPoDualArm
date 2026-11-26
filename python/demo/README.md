# ToPoDualArmの収録・学習・推論

右→左の箱運びを、左右手首の画像と18軸の実測角・指令角で収録し、LeRobotのACTを学習します。収録と推論は同じGUIを使います。

## 準備

ToPoDualArmをクローンし、リポジトリのルートから次を実行します。デモの環境は同じチェックアウトにあるSDKを使用します。

```sh
cd python/demo
uv sync --locked --extra act
```

MacではHomebrewの `eigen@3`、`yaml-cpp`、`tinyxml2` とTkが使えるPythonを用意します。AVFoundationのPython依存はMacで自動的に入ります。Linuxでは `build-essential cmake libeigen3-dev libyaml-cpp-dev libtinyxml2-dev` を用意します。

## 1. 収録

`data/mac-hardware/` に、校正済みの実機直結プロファイル `device.yaml`、左右手首の `cameras-wrists.yaml`、Questと手先の校正 `quest.json` を用意します。ファイル名が異なる場合は `record.yaml` を同じディレクトリの `record-local.yaml` にコピーして編集します。相対パスの基準は設定YAMLのディレクトリです。`root` には未使用の保存先を指定します。

```sh
uv run --locked python viewer/record_quest.py --record record.yaml
```

QuestのGripで操作し、Aで収録開始・採用、Bで棄却します。右→左の成功例を同じデータセットに蓄積します。正面カメラを収録する場合は、カメラ設定に `front` を追加します。

## 2. 学習

```sh
ACT_DEVICE=cuda bash train_act.sh data/right_to_left outputs/right_to_left_act 20000
```

GPUのない環境では `ACT_DEVICE=cpu` を使います。入力は左右手首の320×240画像と18軸の実測角です。ResNet-18の初期重みは `ResNet18_Weights.IMAGENET1K_V1` です。キャッシュにない場合は初回に取得します。`ACT_BACKBONE_WEIGHTS=null` でランダム初期化できます。`train_act.sh` が最終チェックポイントに関節順・収録周波数と、収録の先頭フレームの姿勢を保存します。

## 3. 推論

```sh
uv run --locked python viewer/record_quest.py --inference infer.yaml
```

開始ボタンを押すと学習時の基準姿勢へ移動し、右→左のデモを1回実行します。完了・停止・カメラ欠落時は姿勢を保持します。既存のチェックポイントを使う場合は `infer.yaml` を `infer-local.yaml` にコピーし、重みと初期姿勢を指定します。初期姿勢は `initial_pose_file` または `initial_pose_rad` で指定できます。DPを使う場合は `uv run` に `--extra dp` を加えます。

## LinuxでCLI収録

`tools/episode.py record` で、実機と3台のV4L2カメラから1エピソードを収録できます。`--robot` に実機プロファイル、`--cameras` に3台のデバイスを記したYAML、`--input` に `meta`・`quest`・`glove`・`leader` のいずれかを指定し、`--allow-hardware` を付けます。物理リーダーを使うときは `--leader` も指定します。

収録済みエピソード、機体ごとの校正値、学習済み重みはローカルの `data/` と `outputs/` に保存します。
