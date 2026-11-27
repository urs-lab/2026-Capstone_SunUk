#!/usr/bin/env bash
set -euo pipefail
convoy_stack_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$convoy_stack_dir/convoy_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$convoy_stack_dir/convoy_udp_only.xml"
printf '%s\n' 'Follower localization stack: UDPv4-only participant transport; existing ROS topics/QoS/parameters retained.'
exec bash "$convoy_stack_dir/start_localization.bash" "$@"
