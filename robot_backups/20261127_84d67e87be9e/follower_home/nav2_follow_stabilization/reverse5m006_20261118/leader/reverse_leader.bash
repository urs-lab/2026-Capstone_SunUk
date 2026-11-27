#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS RMW_FASTRTPS_USE_QOS_FROM_XML=0
exec python3 -B "$(dirname -- "${BASH_SOURCE[0]}")/run_leader_distance_trial.py" --run --ack-clear-path-power-cut-ready --direction reverse --distance-m 5 --speed .06 --distance-scale 1 --operator-clear-path
