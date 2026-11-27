# 정량목표1 드라이버 DDS 발행 분리 — 2026-11-18

원본: smooth010/candidate/follower_diagnostics. 정량3의 설정·드라이버 계약으로 교체하지 않았다.
원본 모터 코어, local_avoidance 계약, 하드웨어 YAML, 위치/오돔 변환, 보정 계수는 그대로다.
follower_drive_node.py의 발행 경로만 isolated_telemetry.py로 분리했다.
분리 helper는 11월17일 정량3 성공본의 helper와 동일하다.

사용자 오류 로그: arm 요청 없음·명령수0·장치 상태 양쪽0·측정RPM0 상태에서
status_publish_s=0.725573814, 다음 tick_gap_s=0.72959853,
response_timeout=0.3으로 RESPONSE_TIMEOUT 발생. 수신 버퍼에20bytes가 있었다.
확인된 지연 위치는 상태 DDS publish 호출이다. DDS 내부의 막힘 조건은 특정하지 못했다.

정량1 설정 유지:
- extended follow, 15RPM, session seconds=0.
- 기존 clearance-stop 비활성 설정 유지. 이것을 이번 수정에서 새로 해제한 것이 아니다.
- continuous20=false; 정량3의 10RPM/정지5cm 프로필로 변경하지 않음.
- 모터 command/response/feedback timeout=0.3초.
- 실제 모터 오류/진짜 명령 단절/자식 실패는 기존 오류 처리 또는 오류 정지로 유지.
- 실제 ROS·하드웨어·arm·주행은 도구로 실행하지 않음.

기존 명령은 검증된 새 사본을 실행하도록 연결한다:
```bash
bash ~/nav2_follow_stabilization/smooth010/start_follower_driver.bash
```

직접 경로:
```bash
bash ~/nav2_follow_stabilization/goal1_transport_20261118/start_follower_driver.bash
```

기존 오류가 latch된 드라이버는 Ctrl+C로 종료하고 위 명령으로 다시 실행한다.
센서·AMCL·중계기는 유지한다. 시작 안내에 DDS telemetry isolated in child processes가 있어야 한다.
기존 실행 wrapper는 start_follower_driver.before.bash에 보존했다.
정량1 추종 실행기는 이번에 변경하지 않았다. 콜백 수신 수정이 포함된 정량3 실행기와 구별한다.
정량3 성공 디렉터리·manifest·GitHub 성공 백업도 변경하지 않았다.

검사: validation_tests.log 및 DRIVER_MANIFEST.json. 실물 주행 성공은 별도 확인이 필요하다.
