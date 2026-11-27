# 2026-11-13 실행 안내

## 일반 추종: 기존 노드 유지

사용자가 제시한 공통 환경과 선두 1~5, 후발 1~5, 노트북 RViz 구성을 유지한다.
추종 프로그램은 별도 후발 터미널 6에서 실행한다. 이번 변경을 적용할 때만 먼저
기존 후발 추종 프로그램과 모터 드라이버를 Ctrl+C로 중지하고 아래 순서로 재시작한다.

후발 터미널 1:

```bash
bash ~/nav2_follow_stabilization/smooth010/start_follower_driver.bash
```

모터 드라이버가 다시 시작되면 후발 오뎀 원점도 새로 설정되므로 RViz의
후발 2D Pose Estimate(/follower/initialpose)로 실제 위치와 방향을 재설정한다.

후발 터미널 6:

```bash
bash ~/nav2_follow_stabilization/smooth010/start_follow.bash 0.1
```

입력 준비 후 선두 Nav2 Goal을 지정한다. 총 360초 제한은 제거했다.
Goal 종료, Ctrl+C, 장치/입력 오류 정지는 유지한다. Goal이 끝난 뒤 다른 시험을
시작하려면 추종 프로그램을 다시 실행한다. 이 변경으로 자동 반복 주행하지 않는다.
모터 드라이버와 추종 프로그램은 모두 0초 정책을 사용해야 한다.

## 평가 2: 선두 단독 3m 전진·후진

후발에서 최초 한 번 새 코드 설치(SSH 비밀번호 입력, 설치만 하고 구동하지 않음):

```bash
python3 -B ~/nav2_follow_stabilization/evaluation2/deploy_leader_candidate.py --apply
```

선두 터미널 5(Nav2/gate), 후발 터미널 6(추종), 모든 teleop을 Ctrl+C로 중지한다.
선두 모터·센서·EKF는 유지한다. 다른 센서/위치 노드는 유지 가능하다.
후발은 비무장 상태로 시험 이동 경로 밖에 둔다. 특히 후진 전에 후발을 뒤쪽 3m 경로에서 치운다.

선두에서 전진:

```bash
bash ~/leader_diagnostics/eval2_distance_dbd45e9d9025/forward_3m.bash
```

전진 종료·완전 정지 확인 후, 선두에서 후진:

```bash
bash ~/leader_diagnostics/eval2_distance_dbd45e9d9025/reverse_3m.bash
```

각 명령은 점검 후 START + Enter를 입력해야 이동한다. 전진 최대 0.1m/s, 후진 최대 0.3m/s이며
전진은 실측 기반 보정 거리 3m, 후진은 오뎀 기준 3m에서 정지한다. 실제 이동 거리와 직진 편차는 줄자로 별도 기록한다.
일반 추종의 총 시간 제한만 제거했으며, 평가 2 단독 시험은 60초 고장 제한을 유지한다.
기본 평가 명령은 LiDAR 공간 점검도 유지한다. 장애물 검사 실패 이유는 출력한다.
실제 기체 시험은 사용자 실행 전까지 수행하지 않았다.

0.3m/s 평가 2 실행 전에는 evaluation2/README_KO.md의 전용 35RPM 선두 launch로 터미널 1을 교체한다.

최신 후진 0.5m/s 후보의 실행 경로와 전용 launch는 evaluation2/README_KO.md 마지막 항목을 사용한다.
