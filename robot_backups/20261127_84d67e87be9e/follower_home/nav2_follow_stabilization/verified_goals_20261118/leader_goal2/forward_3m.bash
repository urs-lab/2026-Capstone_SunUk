#!/usr/bin/env bash
set -e
goal2_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$goal2_dir/verify_sources.py"
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
exec python3 -B "$goal2_dir/run_leader_distance_trial.py" \
  --run --ack-clear-path-power-cut-ready --direction forward \
  --distance-m 3 --speed 0.1 --distance-scale 0.935594233291917 --operator-clear-path
