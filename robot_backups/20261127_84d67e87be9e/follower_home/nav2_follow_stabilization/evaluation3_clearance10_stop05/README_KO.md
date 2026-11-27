# 평가 3 전용: 계획 여유 10cm · 정지 기준 5cm

evaluation3_optimization의 20cm·10cm 버전은 그대로 보존했다.
이 폴더는 별도 후보이며 기존 모터·추종·평가2 소스를 덮어쓰지 않는다.
실제 기체를 arm하거나 구동하지 않았다.

- 계획 및 모의 주행 요구 여유: 0.10m. 경로 사전 재검사에는 기존 0.03m 추가 여유 유지.
- 정지 기준: 정지 차체 및 예측 이동 차체의 거리에서 시각·위치 불확실성을 차감한 값이 0.05m 이하.
- 모터 명령도 측정·예측 여유가 모두 0.05m를 초과해야 허용.
- 계획 격자/후보 순서, 모의 주행 간격, 장애물 점 전체와 차체 크기, 10RPM,
  최대 0.025m/s, 360초 평가 시간, 기기 상태·엔코더·명령/피드백 시간 검사는 유지.
- 근접 후 정지 재관측은 새 스캔 5개·기존 최대2초 제한을 유지하며,
  회복 시 요구 여유 0.10m, 예측 여유 0.10m 초과를 요구.
- 계산 최적화는 유지한다. 거리 정책 변경으로 선택 경로는 20cm 버전과 달라질 수 있다.
- 드라이버 상태 continuous_clearance_profile=CONTINUOUS10_STOP05_V1,
  continuous_stop_clearance_m=0.05를 운영 프로그램이 확인하므로 이전 10cm 정지 드라이버는 거절한다.
- --continuous20 이름은 기존 전송 토픽 호환용이다. 실제 새 정책은 상태·계획 config·최종 로그로 명시한다.

## 실행

선두 Nav2/gate·teleop·평가2·후발 일반 추종과 기존 평가3 운영 프로그램을 종료한다.
센서·AMCL·Bridge·중계기는 유지한다. 기존 후발 모터 드라이버를 종료한 후:

```bash
bash ~/nav2_follow_stabilization/evaluation3_clearance10_stop05/start_follower_driver.bash
```

드라이버 재시작으로 오뎀 원점이 바뀌므로 RViz에서 후발 초기 위치·방향을 다시 지정한다.
두 로봇을 정지·정렬하고 현재 기준선을 새로 저장한다:

```bash
bash ~/nav2_follow_stabilization/evaluation3_clearance10_stop05/capture_reference.bash
```

그 뒤 평가 3 실행:

```bash
bash ~/nav2_follow_stabilization/evaluation3_clearance10_stop05/start_evaluation3.bash
```

점검 후 START + Enter로 시작한다. 새 드라이버와 새 운영 프로그램을 함께 사용해야 한다.
기본 정적 TF는 /follower/tf_static_internal이다. 공개 TF 구성은
CONVOY_TF_STATIC_TOPIC=/tf_static으로 지정한다. 이전 버전으로 돌아가려면 두 프로그램을
종료하고 evaluation3_optimization 폴더의 드라이버·기준선 캡처·운영 프로그램을 사용한다.

## 검증

clearance_tests.log: 10cm 계획, 5cm 차체 정지 경계, 이전 드라이버 거절, 실제 저장 장면 계획, 실행 스크립트.
policy_regression_tests.log: 연속 곡선·센서 시각·휠 한도·재관측·양쪽 정지 회귀 검사.
driver_policy_tests.log: 실제 현행 드라이버 사본에서 5cm 경계 입력 거절, 7cm 여유 주행/정지 모의,
기기 상태 오류·피드백·명령 시간 초과 검사. 모두 가짜 장치/ROS이며 실물 검증이 아니다.

최종 검증: 새 정책·실행 9개, 제어 회귀 37개, 드라이버·오류 검사 86개 통과(총132개).

## 필수 설정 파일 누락 수정

driver/follower_hardware.yaml과 follower_odometry_calibration.json을 기존 smooth010에서
바이트 단위 동일하게 복사하고 driver_settings_manifest.json에 SHA256을 기록했다.
실제 드라이버 코드의 설정 로드·검증 및 거리 보정 로드를 ROS/시리얼 없이 실행해 3개 검사를 통과했다.
물리 포트 열기·arm·모터 구동은 수행하지 않았다.
