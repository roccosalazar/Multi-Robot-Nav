#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <tag> [spawn_robot_launch_args...]" >&2
  echo "Example: $0 baseline generate:=false" >&2
  exit 2
fi

TAG="$1"
shift
ROBOT_ARGS=("$@")

cleanup() {
  kill "${RPID:-}" "${WPID:-}" >/dev/null 2>&1 || true
  pkill -f "ign gazebo" || true
  pkill -f "spawn_world.launch.py" || true
  pkill -f "spawn_robot.launch.py" || true
}
trap cleanup EXIT

cd /home/ubuntu/Multi-Robot-Nav

# source_workspaces.sh uses relative paths and may touch unset vars in sourced scripts
set +u
source ./source_workspaces.sh
set -u

export DISPLAY=:1

pkill -f "ign gazebo" || true
pkill -f "spawn_world.launch.py" || true
pkill -f "spawn_robot.launch.py" || true
sleep 2

ros2 launch musketeers_bringup spawn_world.launch.py world:=warehouse > "/tmp/${TAG}_world.log" 2>&1 &
WPID=$!
sleep 12

ros2 launch musketeers_bringup spawn_robot.launch.py world:=warehouse "${ROBOT_ARGS[@]}" > "/tmp/${TAG}_robot.log" 2>&1 &
RPID=$!
sleep 25

timeout 12s ign topic -e -n 12 -t /world/warehouse/stats > "/tmp/${TAG}_stats.txt" || true

awk -v tag="$TAG" '/real_time_factor:/{sum+=$2;n++; if(min==0||$2<min)min=$2; if($2>max)max=$2} END {if(n>0) printf("%s_RTF avg=%.4f min=%.4f max=%.4f n=%d\n", tag, sum/n, min, max, n); else printf("%s_RTF no-samples\n", tag)}' "/tmp/${TAG}_stats.txt"
echo "${TAG}_RTF_samples_start"
grep real_time_factor "/tmp/${TAG}_stats.txt" | sed -n '1,12p'
echo "${TAG}_RTF_samples_end"

echo "Artifacts:"
echo "  /tmp/${TAG}_world.log"
echo "  /tmp/${TAG}_robot.log"
echo "  /tmp/${TAG}_stats.txt"
