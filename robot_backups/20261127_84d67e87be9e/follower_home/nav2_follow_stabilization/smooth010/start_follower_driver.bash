#!/usr/bin/env bash
set -e
# Goal1 settings retained; DDS telemetry no longer runs on the hardware tick.
exec bash "$HOME/nav2_follow_stabilization/goal1_transport_20261118/start_follower_driver.bash"
