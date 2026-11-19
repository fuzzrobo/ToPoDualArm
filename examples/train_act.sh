#!/usr/bin/env bash
# ローカルのエピソードでACTの学習→保存を確認する。Hubには接続しない。
set -eu
dataset=${1:?datasetのパスが必要です}
output=${2:?出力先が必要です}
steps=${3:-4}
export HF_HUB_OFFLINE=1 WANDB_MODE=disabled
uv run --locked --extra act --extra record lerobot-train \
  --dataset.repo_id=local/topoarm --dataset.root="$dataset" --dataset.video_backend=pyav \
  --policy.type=act --policy.device="${ACT_DEVICE:-cpu}" --policy.push_to_hub=false \
  --policy.pretrained_backbone_weights=null --policy.chunk_size=4 --policy.n_action_steps=4 \
  --policy.dim_model=64 --policy.dim_feedforward=128 --policy.n_heads=4 \
  --policy.n_encoder_layers=1 --policy.n_decoder_layers=1 --policy.n_vae_encoder_layers=1 \
  --steps="$steps" --batch_size=2 --num_workers=0 --save_freq="$steps" --log_freq=1 \
  --wandb.enable=false --output_dir="$output"
# ACTの重みには関節名が含まれないので、学習データの軸順とfpsを一緒に渡す。
cp "$dataset/meta/info.json" "$output/checkpoints/last/pretrained_model/topoarm_schema.json"
