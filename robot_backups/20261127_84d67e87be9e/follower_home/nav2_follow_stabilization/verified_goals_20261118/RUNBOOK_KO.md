# 정량목표 1·2·3 실행표 — 2026-11-18

정량1: 선두 Nav2 Goal 주행 + 후발 실측 경로 추종(최대0.1m/s,15RPM).
정량2: 선두 단독3m 전진을3회. 전진0.1m/s, 보정계수0.935594233291917.
정량3: 후발 장애물 회피 → 지도/선두 경로 복귀 → 추종 재개(여유10cm,정지5cm,10RPM).
세션 제한0은 모든 오류 감시가 없어지는 뜻이 아니다. 기동 직전 정렬 지연도 후발 비무장으로 대기한다.

이번 변경의 실물 주행은 도구로 시작하지 않았다.
정량3은11월17일 사용자 확인 성공본을 그대로 유지했다.
정량1은11월18일 드라이버 발행 분리·운영 실행기 콜백/상태 분리·시각 불일치 대기 개선 후 오프라인 검증 상태다.
정량2는 기존 보정/속도/거리 코드를 보존했다. 현재 선두 SSH는 도구 환경에서 차단돼 현장 파일을 재조회하지 못했다.

## 모든 ROS 터미널 공통 환경 — 선두/후발

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
```

## 정량1·3 공통 선두 — sunuk@ubuntu

선두1 모터·센서·EKF:
```bash
ros2 launch ~/leader_diagnostics/leader_nav_yaw_calibrated.launch.py allow_motion:=true
```

선두2 지도·AMCL:
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/sunuk/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml \
  params_file:=/home/sunuk/leader_diagnostics/nav2_015_rotation_shim_check.yaml \
  use_sim_time:=false autostart:=true
```

선두3 Bridge:
```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/convoy_udp_only_pilot.xml" \
python3 -B ~/leader_diagnostics/leader_bridge_latest_candidate.py \
  --ros-args -p map_frame:=map -p base_frame:=base_link -p publish_rate:=20.0
```

선두4 유니캐스트:
```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/convoy_udp_only_pilot.xml" \
python3 -B ~/leader_diagnostics/convoy060_dfa647fff6de/convoy_unicast_relay.py \
  --role leader --peer 10.253.245.192
```

## 정량1·3 공통 후발 — sunuk@follower

후발 모터는 아래 평가별 명령을 먼저 실행한다. 모터 드라이버는 하나만 실행한다.

후발2 센서·EKF·지도·AMCL·Bridge:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_follower_localization.bash
```

후발3 유니캐스트:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_follower_unicast.bash
```

센서 wrapper는 정량3 성공본의 TF·센서·AMCL 설정을 보존한 localization_startup_20261118을 실행한다.
지도 서버 bond 기동 실패를 확인해 센서 후4초 기동·lifecycle bond20초·읽기 전용 준비 확인을 추가했다.
이 bond 값은 이후 localization lifecycle heartbeat 손실 감지에도 영향을 준다. 모터0.3초/입력0.8초 감시는 그대로다.
중계기 wrapper는 정량3 성공본을 그대로 실행한다.
이전 success_tf_recovery/start_localization_udp.bash 또는 private TF stack을 중복 실행하지 않는다.
센서/Bridge를 별도 터미널에 추가로 띄우지 않는다.

## 노트북 RViz

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
rviz2 -d ~/convoy_view.rviz
```

Fixed Frame=map. 선두2D Pose 토픽=/initialpose, 후발=/follower/initialpose.
LOCALIZATION_ACTIVE 메시지가 나온 뒤 실제 차체 위치와 방향을 지정한다.
이전에 AMCL inactive 상태에서 보낸 위치가 적용됐다고 가정하지 않는다.
FOLLOWER_MAP_ODOM_READY까지 확인한다. 자동으로 원점/이전 초기 자세를 강제하지 않는다.
후발에서 입력 확인:

```bash
ros2 lifecycle get /follower/amcl
timeout 10 ros2 topic echo /follower/odom_map --once --qos-reliability best_effort
```

## 정량목표1

선두1~4 유지. 선두5 Nav2·gate:
```bash
bash ~/leader_diagnostics/nav2_follow_stability_da0ab05848da/start_leader_nav2.bash
```

후발1 전용 드라이버:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_goal1_driver.bash
```

후발4 추종 실행:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_goal1_follow.bash 0.1
```

초기 위치·입력 준비 후 RViz에서 선두 Nav2 Goal을 지정한다.
자동 Goal 감시 모드이므로 평가3의 START를 입력하는 흐름과 구별한다.
현재 버전 식별: operator_variant=GOAL1_POSE_SCAN_WAIT_20261118.
지도/스캔 시각이 어긋나면 정지 명령과 gate 대기를 유지하며 정렬 복구 시 재개한다.
0.4초 정렬 기준·0.8초 입력 기준은 유지한다. 모터 상태 지연도 동일 세션으로 대기한다.
미래 시각/잘못된 기하/실제 raw·scan 손실/모터 fault/세션 변경은 종료할 수 있다.
기존 거리정지 해제 설정과 smooth-follow 속도 설정은 바꾸지 않았다.

기존 명령도 새 사본에 연결한다:
```bash
bash ~/nav2_follow_stabilization/smooth010/start_follower_driver.bash
bash ~/nav2_follow_stabilization/success_tf_recovery/start_follow.bash 0.1
```
위 두 명령은 서로 별도 터미널이며 새 통합 명령과 중복 실행하지 않는다.

## 정량목표3

선두 Nav2·gate 및 정량1 후발 추종을 Ctrl+C로 종료한다.
양쪽 정지를 확인한 뒤 후발 정량1 드라이버도 종료하고 아래 드라이버로 교체한다.
선두1~4와 후발 센서·유니캐스트는 유지한다.

후발1:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_goal3_driver.bash
```

실제 초기 위치·방향을 지정하고 양쪽 정지·후발 비무장에서 기준선 저장:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/capture_goal3_reference.bash
```

REFERENCE_JSON 저장 성공 후:
```bash
bash ~/nav2_follow_stabilization/verified_goals_20261118/start_goal3.bash
```

START + Enter. 기존 성공본 goal3_success_tf02524/start_evaluation3_ekf_wait.bash로 연결한다.
정량3 성공본 소스·설정·hash는 변경하지 않았다.

## 정량목표2 — 선두에서만 실행

Nav2·gate, 정량1 추종, 정량3 평가, teleop 등 주행 명령 발행기를 종료한다.
후발은 정지·비무장 상태로 경로 밖에 둔다. 선두 모터·센서·EKF(선두1)는 유지한다.

기존 선두 설치 경로가 있는 경우(기존 명령 유지):
```bash
python3 -B ~/leader_diagnostics/eval2_distance_dbd45e9d9025/run_leader_distance_trial.py \
  --run --ack-clear-path-power-cut-ready --direction forward \
  --distance-m 3 --speed 0.1 --distance-scale 0.935594233291917 --operator-clear-path
```

매회 실제 출발/도착점을 기록하며 위 명령을3회 각각 실행한다. START + Enter는 매회 입력한다.
매 실행마다 현재 오돔을 새 출발 기준으로 사용하며 목표값에 이전3m를 누적하지 않는다.
물리적3m 충족 여부는 외부 측정으로 확인한다. 위 계수는 기존 사용자 실측 보정값을 보존했다.

기존 파일이 없을 때만 사용할 동일 소스 휴대용 사본도 준비했다.
후발에서 한 번 복사(설정 변경/주행 없음):
```bash
scp -r ~/nav2_follow_stabilization/verified_goals_20261118/leader_goal2 \
  sunuk@10.253.245.81:~/leader_diagnostics/goal2_verified_20261118
```
새 경로가 이미 있을 때 중첩 복사를 반복하지 않는다. 선두에서:
```bash
bash ~/leader_diagnostics/goal2_verified_20261118/forward_3m.bash
```

## 전환 요약

| 목표 | 선두 Nav2·gate | 후발 드라이버 | 후발 실행기 |
|---|---|---|---|
|1|실행|goal1,15RPM|start_goal1_follow|
|2|종료|정지·비무장|종료, 선두 거리 실행기만 사용|
|3|종료|goal3,10RPM|capture 후 start_goal3|

모터 드라이버·평가 실행기·센서 stack·유니캐스트를 각각 중복 실행하지 않는다.
이 문서는 명령을 자동 실행하지 않는다. 실제 주행은 사용자가 준비 후 시작한다.
