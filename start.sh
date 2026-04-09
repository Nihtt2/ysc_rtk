#!/usr/bin/env bash
set -euo pipefail

source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate unitree

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python new_rtk.py \
    --transport udp \
    --waypoint-file 1.json \
    --start-walk-angle-deg 6 \
    --arrival-radius 0.4 \
    --arrival-radius-max 0.8 \
    --coarse-threshold 40 \
    --fine-threshold 6 \
    --nav-max-vx 0.5 \
    --nav-min-vx 0.2 \
    --max-yaw-rate 0.4 \
    --nav-long-segment-step-dist 4 \
    --nav-close-stage-dist 0.6 \
    --nav-long-segment-trigger-dist 10000 \
    --nav-decel-start-dist 2 \
    --nav-min-vx-reached-dist 0 \
    --gnss-antenna-x -0.58 \
    --gnss-antenna-y 0 \
    --gnss-antenna-z 0.175 \
    --gnss-rolling-calibration 1
