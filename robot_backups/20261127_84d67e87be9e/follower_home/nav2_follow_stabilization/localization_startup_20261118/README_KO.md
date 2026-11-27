# 후발 위치추정 기동 확인 — 2026-11-18

실제 실패 로그:
- .ros/log/lifecycle_manager_7033_1794991498876.log:
  map_server 활성화 후4초 bond 확인 실패 → bringup 중단. AMCL 활성화 단계에 도달하지 못함.
- .ros/log/amcl_7024_1794991498854.log:
  initialPoseReceived 뒤 "AMCL is not yet in the active state". Setting pose 기록 없음.
- map_server 로그에는 Activating/Create bond가 있음.

이 경우 AMCL은 지도 위치를 TF로 발행하지 못하므로 map→follower/odom이 연결되지 않는다.
map_server가 active인 것만으로 전체 위치추정이 준비됐다고 볼 수 없다.

변경:
1. 센서·EKF 시작4초 뒤 지도/AMCL/lifecycle manager를 시작해 초기 동시 기동 부하를 나눈다.
2. localization manager의 bond_timeout을 기본4초에서20초로 설정한다. bond를 비활성화하지 않는다.
   이 값은 기동 bond 형성과 이후 lifecycle heartbeat 손실 감지에도 영향을 준다.
   원본0.8초 주행 입력 신선도와0.3초 모터 감시는 변경하지 않았다.
3. 읽기 전용 watcher가 map_server·AMCL active와 manager/is_active=True를 확인해야
   LOCALIZATION_ACTIVE를 표시한다. 그 뒤 실제 후발 초기 위치를 지정한다.
   map odom의 원본 시각과 수신 나이까지 확인하면 FOLLOWER_MAP_ODOM_READY를 표시한다.
4. watcher는 publisher, arm, ChangeState 요청이 없으며 실패한 manager를 우회해 AMCL만 강제활성화하지 않는다.
5. AMCL 설정·스캔 TF·EKF 수학·Bridge는 성공본과 동일하고 TF0.25.24 라이브러리도 바이트 동일하다.
6. 성공본 goal3_success_tf02524 자체는 변경하지 않았다.

이번 파라미터는 확인된 기동 지연을 처리하기 위한 별도 실행본이며 실제 기동 확인이 필요하다.
LOCALIZATION_NOT_READY가 계속되면 준비되지 않은 상태다. 상태 메시지를 성공으로 바꾸지 않는다.

실행 중인 평가를 종료하고 양쪽 정지·후발 비무장에서 센서/위치추정 통합 터미널만 Ctrl+C로 종료한다.
모터 드라이버·중계기는 유지한다. 다시 실행:

```bash
bash ~/nav2_follow_stabilization/localization_startup_20261118/start_localization.bash
```

LOCALIZATION_ACTIVE가 출력된 뒤 RViz /follower/initialpose로 실제 위치·방향을 다시 지정한다.
이전 inactive 상태에서 보낸 초기 위치가 적용됐다고 가정하지 않는다.
FOLLOWER_MAP_ODOM_READY를 확인한 뒤 해당 평가를 시작한다.

유니캐스트 로그의 receive_rejected는 누적값이다. 제공된 마지막 구간에서는1317에 고정됐고,
received_1/2/3은5초마다 약100씩 증가해 정상 수신이 재개됐다. 과거 거절의 개별 원인은 카운터만으로 특정하지 않는다.
유니캐스트 중계기는 map→follower/odom TF를 생성하지 않는다.

Nav2 동작 근거: https://github.com/ros-navigation/navigation2/blob/humble/nav2_lifecycle_manager/src/lifecycle_manager.cpp

## 11월18일 추가 확인 및 복구

새 실행본을 만들 때 Bridge의 로컬 의존 follower_input_guard.py를 복사하지 않은 오류가 있었다.
실제 launch 로그에 Bridge pid7477 exit1, 조회 결과 /follower/odom_map publisher0으로 확인됐다.
별도 인터프리터에서도 ModuleNotFoundError를 재현했다. 원본 성공본의 동일 helper를 복사했고,
이제 SOURCE_MANIFEST에도 포함했다. 새 interpreter와 다른 cwd에서 Bridge import를 검사하는 시험을 추가했다.
설정/준비 정책/실제 import 경로 검사10개 통과. 이 검사는 ROS/실물 주행 실행이 아니다.

이미 센서·AMCL이 active이고 Bridge만 종료된 현재 상태에서는 나머지를 재시작하지 않고:
```bash
bash ~/nav2_follow_stabilization/localization_startup_20261118/start_bridge_only.bash
```
이 명령은 이미 살아 있는 Bridge와 중복 실행하지 않는다.
센서 통합을 나중에 전체 재시작할 때는 복구된 Bridge도 함께 정상 시작된다.


2026-11-18 19:24 시작 실패 추가 확인: 지도 읽기 성공 후 change_state 응답 timeout,
manager는 Configuring map_server에서 멈췄고 AMCL configure/activate 이전에 사용자 종료.
이 실패는 bond 단계 전이므로 bond_timeout20초만으로 해결된다고 단정할 수 없음.
start_localization_udp.bash 추가: 기존 성공 recovery XML과 동일한 UDPv4 전용 프로파일 명시.
공통 진입점: bash ~/nav2_follow_stabilization/verified_goals_20261118/start_follower_localization.bash
다른 통합 센서/AMCL/Bridge 및 start_bridge_only 실행은 종료한 상태에서 이것 하나만 실행.
LOCALIZATION_ACTIVE 확인 후 RViz 초기위치, FOLLOWER_MAP_ODOM_READY 확인 후 운영자 시작.
서비스 응답 timeout의 DDS 내부 원인은 아직 입증되지 않음. 실제 재기동 성공은 사용자 확인 필요.
