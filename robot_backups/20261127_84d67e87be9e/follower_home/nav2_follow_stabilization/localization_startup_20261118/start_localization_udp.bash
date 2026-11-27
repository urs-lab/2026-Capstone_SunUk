#!/usr/bin/env bash
set -e
stack_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$stack_dir/convoy_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$stack_dir/convoy_udp_only.xml"
printf '%s\n' '후발 정량1/3 공통: 성공본 UDPv4 설정·TF0.25.24·센서 후 위치추정 기동·준비 상태 확인. Bridge 포함 1개.'
exec bash "$stack_dir/start_localization.bash" "$@"
