# 0.1m/s 상한의 별도 추종 설정

성공한 0.06 설정은 수정하지 않았다. 현재 운용 코드/로그 320개를
../backups/follow_006_success_1794471911343705659에 백업하고 해시를 확인했다.
새 설정은 speed010/candidate/follower_diagnostics에만 있다.

0.1m/s 직진에는 반경 0.065m 기준 약 14.69RPM이 필요하다. 이 설정은
명시적 --ack-extended-follow-speed와 15RPM 드라이버를 함께 요구한다.
기존 설정을 10RPM인 채로 두고 속도 옵션만 변경하는 방식이 아니다.
정수 RPM 반올림은 기존 방식이므로 0.1 직진은 원시 [-15,15]RPM 명령이 된다.
물리적 속도나 제동 성능을 이 계산만으로 보증하지 않는다.

선두 gate도 별도 새 디렉터리에 설치한다. 기존 선두 파일은 덮어쓰지 않는다.
출발 대기 시 선두 crawl은 0.04m/s를 유지한다. 이후 상한은 0.1m/s이다.
회전 때 15RPM을 넘으면 선속도와 각속도를 함께 낮추며, 선두에는 실제
제한된 후발 요청 속도를 전달한다. 거리/회전에 따라 실제 속도는 더 낮다.

엔코더/장치 알람, 피드백/명령 시간 제한, 360초 한도, 경로 입력의 10초 대기
및 검증 후 복구를 유지한다. 사용자 선택의 후발 거리 정지 해제도 유지한다.
모터 드라이버의 EEPROM/엔코더 PPR/하드웨어 최대 RPM 설정은 쓰지 않는다.

## 적용 순서

먼저 기존 Goal을 취소하고 후발 추종 프로그램, 후발 모터 드라이버,
선두 Nav2 주행 launch를 각각 Ctrl+C로 종료한다. 텔레오프도 종료한다.
센서·지도·AMCL·Bridge·unicast relay는 기존 구성을 유지한다.

후발에서 파일만 선두로 복사한다. SSH 비밀번호 입력이 필요할 수 있다.
이 명령 자체는 로봇 프로세스를 실행하거나 arm하지 않는다.

```bash
python3 -B ~/nav2_follow_stabilization/speed010/deploy_leader_candidate.py --apply
```

선두에서 새 Nav2/gate를 실행한다.

```bash
bash ~/leader_diagnostics/nav2_follow_stability_c12185ae874d/start_leader_nav2.bash
```

후발 모터 터미널에서:

```bash
bash ~/nav2_follow_stabilization/speed010/start_follower_driver.bash
```

드라이버 재시작으로 odom 기준이 바뀔 수 있으므로 후발 초기 Pose를 실제
위치에 맞게 다시 지정한다. TF/위치가 안정된 뒤 후발의 다른 터미널에서:

```bash
bash ~/nav2_follow_stabilization/speed010/start_follow.bash 0.1
```

입력 준비 후 첫 Goal은 장애물 없는 직선 약 1m로 지정한다. 이번 설정은
실제 주행으로 아직 검증되지 않았다. 급회전/알람/정지 불능이면 전원을 차단한다.
0.08 상한으로 먼저 비교하려면 마지막 인자만 0.08로 지정한다.

## 검증 및 한계

- regression.log: 관련 회귀 검사 187개 통과.
- extended_operator_tests.log: 추가 증속 흐름과 관련 검사 30개 통과(위 검사와 중복 포함).
- bounds_check.json: 0.08/0.1의 명령 1,620쌍에서 선속도·각속도·전진 방향·15RPM 제한 확인.
- deployment_test.log: 로컬 모의 SSH에서 새 디렉터리 설치, 재실행 확인,
  기존 파일 보존, 변경된 기존 설치 덮어쓰기 거부를 확인.
- bash -n과 Python 문법 검사 통과. 실제 선두 복사와 모터 구동은 하지 않았다.

Nav2 controller/velocity_smoother 자체의 속도 제한은 기존 선두 YAML을 그대로
사용한다. 그 설정이 0.1보다 낮으면 gate 상한만 올려도 실제 속도는 올라가지 않는다.

## 되돌리기

증속 추종/드라이버와 새 선두 Nav2 launch를 종료한다. 선두는 이전에 사용한
0.06 launch로 실행하고, 후발은 speed010 경로가 아닌 기존
~/nav2_follow_stabilization/candidate/follower_diagnostics의 드라이버/추종
프로그램을 사용한다. 드라이버 --trial-max-rpm 10, 추종 --follow-max-linear 0.06을
사용하며 --ack-extended-follow-speed는 넣지 않는다.
기존 10RPM 드라이버와 새 15RPM 운용 옵션이 섞이면 사전 점검에서 차단된다.
