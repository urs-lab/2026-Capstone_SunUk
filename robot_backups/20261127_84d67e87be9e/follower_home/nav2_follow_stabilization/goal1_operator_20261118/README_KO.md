# 정량목표1 운영 실행기 — 2026-11-18

원본 smooth010 제어 함수와 원본 디렉터리는 보존했다. 별도 사본에 다음을 적용했다.

1. 한 SingleThreadedExecutor와 고정 timeout으로 콜백 큐를 이어서 처리.
2. 표시 상태 DDS 발행은 자식 프로세스. 실제 제어 명령 발행 주체는 유지.
3. 지도 위치/스캔 시각 차이0.4초 검사·입력 나이0.8초 검사는 그대로.
   시각 차이/과거 지도 위치 지연이면 fresh raw/scan 원본 시각으로 STOP만 유지한다.
   오래된 지도 위치로 장애물 투영이나 DRIVE 계산을 하지 않는다.
4. 새 지도 위치·스캔·선두/경로 입력,1초 안정·3개 원본 시각·후발 측정 정지 확인 후 재개.
5. seconds=0 프로필은 이 복구의10초 제한도 없앴다. 유한 프로필의 기본10초 제한은 유지.
   모터 상태 수신 또는 원본 상태 시각 지연도 같은 속도0 대기를 사용한다.
   새 상태의 원본 시각0.3초 이내·동일 epoch·후발 측정 정지가 확인돼야 복구 검증을 진행한다.
6. 지도위치의 미래 시각·잘못된 프레임/기하·실제 raw/scan 손실·모터 fault·epoch 변경은 계속 종료한다.
   driver command/feedback/response0.3초 감시는 변경하지 않는다.

0.1m/s·15RPM·smooth-follow·direct-unicast·차체 내부점 제외·기존 거리정지 해제 설정을 유지했다.
Pure Pursuit/갭 제어/오돔/모터 계약·지도 투영 함수 수학은 원본과 같다.
정량3의10RPM/정지5cm 프로필과 혼합하지 않는다.

실행:
```bash
bash ~/nav2_follow_stabilization/goal1_operator_20261118/start_follow.bash 0.1
```

기존 success_tf_recovery/start_follow.bash 0.1 명령도 smooth010 wrapper를 통해 위 사본으로 연결한다.
실행 중이던 프로세스에는 파일 수정이 소급되지 않는다. 기존 추종만 종료 후 다시 실행한다.
완료 출력의 operator_variant=GOAL1_POSE_SCAN_WAIT_20261118로 식별한다.

오프라인 검증은 goal1_regression_tests.log, goal1_flow_regressions.log.
실제 ROS 주행 성공·정식 평가 합격은 새 실행으로 별도 확인해야 한다.
