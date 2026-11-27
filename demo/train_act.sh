#!/usr/bin/env bash
# ローカルのLeRobotデータセットからACTを学習する。
set -eu
dataset=${1:?datasetのパスが必要です}
output=${2:?出力先が必要です}
steps=${3:-20000}
demo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec uv run --project "$demo_dir" demo train "$dataset" "$output" --steps "$steps"
