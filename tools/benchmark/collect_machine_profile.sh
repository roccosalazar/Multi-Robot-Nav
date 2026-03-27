#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-/tmp/machine_profile_$(date +%Y%m%d_%H%M%S).txt}"

{
  echo "=== DATE ==="
  date -Is
  echo

  echo "=== CPU ==="
  lscpu || true
  echo

  echo "=== RAM ==="
  free -h || true
  echo

  echo "=== DISK ==="
  lsblk -o NAME,SIZE,TYPE,MOUNTPOINT,MODEL || true
  echo

  echo "=== GPU_LSPCI ==="
  lspci | grep -iE 'vga|3d|display' || true
  echo

  echo "=== GPU_NVIDIA_SMI ==="
  nvidia-smi -L || true
  nvidia-smi || true
  echo

  echo "=== DRI_AND_GROUPS ==="
  id || true
  ls -ln /dev/dri || true
  echo

  echo "=== OS ==="
  uname -a || true
  cat /etc/os-release || true
} > "$OUT"

echo "Wrote profile to: $OUT"
