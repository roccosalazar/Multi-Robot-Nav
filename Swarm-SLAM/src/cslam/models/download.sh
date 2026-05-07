#!/usr/bin/env bash

set -euo pipefail

# Download trained CosPlace checkpoints from:
# https://github.com/gmberton/CosPlace
# The default parameters are for ResNet-18 with dimension=64.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SCRIPT_DIR

download_model() {
  local output_name="$1"
  local backbone="$2"
  local descriptor_dim="$3"

  OUTPUT_NAME="$output_name" BACKBONE="$backbone" DESCRIPTOR_DIM="$descriptor_dim" python3 - <<'PY'
import os
from pathlib import Path

import torch

script_dir = Path(os.environ["SCRIPT_DIR"])
output_name = os.environ["OUTPUT_NAME"]
backbone = os.environ["BACKBONE"]
descriptor_dim = int(os.environ["DESCRIPTOR_DIM"])
output_path = script_dir / f"{output_name}.pth"

if output_path.exists():
    print(f"Skipping {output_path.name} (already exists)")
    raise SystemExit(0)

print(f"Downloading {output_path.name} from gmberton/cosplace")
model = torch.hub.load(
    "gmberton/cosplace",
    "get_trained_model",
    backbone=backbone,
    fc_output_dim=descriptor_dim,
    trust_repo=True,
)
torch.save(model.state_dict(), output_path)
print(f"Saved {output_path}")
PY
}

case "${1:-default}" in
  default|resnet18_64)
    download_model "resnet18_64" "ResNet18" "64"
    ;;
  resnet101_512)
    download_model "resnet101_512" "ResNet101" "512"
    ;;
  all)
    download_model "resnet18_64" "ResNet18" "64"
    download_model "resnet101_512" "ResNet101" "512"
    ;;
  *)
    echo "Usage: $0 [default|resnet18_64|resnet101_512|all]" >&2
    exit 1
    ;;
esac
