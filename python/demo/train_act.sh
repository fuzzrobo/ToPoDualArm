#!/usr/bin/env bash
# ローカルのLeRobotデータセットからACTを学習する。
set -eu
dataset=${1:?datasetのパスが必要です}
output=${2:?出力先が必要です}
steps=${3:-20000}
demo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run=(uv run --project "$demo_dir" --locked --extra act)
export HF_HUB_OFFLINE=1 WANDB_MODE=disabled
features=$("${run[@]}" python "$demo_dir/tools/demo_dataset.py" "$dataset")

"${run[@]}" lerobot-train \
  --dataset.repo_id=local/topoarm --dataset.root="$dataset" --dataset.video_backend=pyav \
  --policy.type=act --policy.device="${ACT_DEVICE:-cpu}" --policy.push_to_hub=false \
  "--policy.input_features=$features" --policy.vision_backbone=resnet18 \
  "--policy.pretrained_backbone_weights=${ACT_BACKBONE_WEIGHTS:-ResNet18_Weights.IMAGENET1K_V1}" \
  --steps="$steps" --batch_size=8 --num_workers=0 --save_freq="$steps" --log_freq=100 \
  --wandb.enable=false --output_dir="$output"
# ACTの重みには関節名が含まれないので、学習データの軸順とfpsを一緒に渡す。
checkpoint="$output/checkpoints/last/pretrained_model"
cp "$dataset/meta/info.json" "$checkpoint/topoarm_schema.json"
"${run[@]}" python "$demo_dir/tools/demo_dataset.py" "$dataset" --initial-pose "$checkpoint/initial_pose.json"
