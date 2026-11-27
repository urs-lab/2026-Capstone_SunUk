# 정량목표1 GitHub 성공 백업과 현재 실행본 비교 — 2026-11-18

## 비교한 실제 GitHub 버전

- 저장소: https://github.com/wpdhks1029/capstone_goal1
- 현재 main: b50321217157ebafbc302c7d028ddb7c0bb24591
- GitHub commit tree: e1864296496d98ce091ac3fc37640b58c91ffb9f
- 로컬 성공 백업 전체의 파일·실행권한·디렉터리를 Git 규칙으로 해시한 트리도 동일하다.
  따라서 인터넷의 백업과 다른 임의의 로컬 사본을 비교한 것이 아니다.
- 별개의 정량목표3 어제 성공 커밋은 c73ac149cd30e003530ff3834de93a8c19d2873e다.
  정량3의 성공 실행기를 정량1 실행기와 같은 코드라고 취급하지 않는다.

## 파일별 전수 비교 결과

원본 저장소 총1,347개 파일:

| 분류 | 개수 | 확인 결과 |
|---|---:|---|
| 후발 HOME 파일 일치 | 1,014 | GitHub 성공 백업과 바이트 단위 동일 |
| 후발 HOME 변경 | 1 | smooth010/start_follower_driver.bash. 오늘 별도 텔레메트리 드라이버 사본으로 연결한 wrapper |
| 후발 HOME 누락 | 0 | 없음 |
| 선두 수집본 파일 | 326 | GitHub 백업과 로컬 보관본 일치는 트리 해시로 확인. 현재 선두 SSH 파일은 이번 작업에서 조회하지 않았으므로 현재 선두와 일치한다고 주장하지 않음 |
| 저장소 자체 문서·manifest | 6 | 실행 HOME 파일과 구별 |

smooth010/candidate/follower_diagnostics의 원본324개 파일은 현재 원본 디렉터리와 전부 동일하다.
여기에는 run_local_avoidance_trial.py, nav2_follow_safety.py, follow_reference_recovery.py,
추종 코어, 하드웨어 YAML, 오돔 보정, driver 계약과 테스트가 포함된다.

현재 활성 드라이버 사본(goal1_transport_20261118/driver)은 원본324개 중
323개가 동일하고 follower_drive_node.py의 DDS 텔레메트리 분리만 다르다.
추가 helper isolated_telemetry.py 등은 원본에 없던 추가 파일이다.
원본의 15RPM·0.1m/s용 extended follow·세션0·모터 감시0.3초 정책은 유지했다.
어제 성공한 정량3의 continuous20/10RPM/정지5cm 계약으로 정량1을 교체하지 않았다.

주의: 현재 사용자가 실행한 후발 센서 wrapper는 정량3 성공본의 TF0.25.24 적용
start_localization.bash다. 원본 정량1 백업의 센서 wrapper와 실행 경로가 같다는 뜻은 아니다.
선택된 새 wrapper는 기존 센서·EKF·AMCL 파라미터와 public TF를 사용하고 AMCL 라이브러리 경로를 바꾼다.
정량1 프로그램 원본 일치와 전체 프로세스 실행 환경 일치는 별개로 기록한다.

## 이번 오류를 만드는 코드

nav2_follow_safety.py build_scene:

1. follower map pose와 scan이 각각0.8초 이내인지 검사한다.
2. 두 원본 시각 차이의 절댓값이0.4초를 초과하면 FOLLOW_POSE_SCAN_TIME_MISMATCH를 발생시킨다.
3. 현재 위치/장애물 clearance 설정과 별도로 이 검사를 항상 수행한다.

운영 실행기의 follow_scene은 /follower/odom_map과 /follower/scan으로 이 함수를 호출한다.
이 이름의 POSE는 /follower/odometry/filtered 메시지 자체가 아니라 지도 기준 후발 위치다.
follow_reference_recovery.py의 recoverable_reference_loss에는 이 오류가 등록돼 있지 않다.
따라서 선두 참조 손실 대기로 처리되지 않고 외부 ValueError 처리로 세션이 종료된다.
follow-only이므로 local core의 FILTERED_ODOM_SCAN_BRACKET_MISSING 복구 분기도 이 오류를 처리하지 않는다.

이 세 가지 정책은 GitHub 원본과 현재 코드가 같다. 오늘 모터 발행 수정이 이 조건을 새로 만든 것은 아니다.

## 최신 로그로 확인한 실제 실행 순서

로그: smooth010/candidate/follower_diagnostics/logs/local_avoidance_trial_1794989578439735516.jsonl

| 관측 | 지도 위치 원본 시각(ns) | 스캔 원본 시각(ns) | 차이 |
|---|---:|---:|---:|
| 종료 직전 제어 | 1794989624079753805 | 1794989624296378000 | 0.216624195초 |
| 마지막 제어 | 1794989624079753805 | 1794989624399321000 | 0.319567195초 |

이후 ros_ns=1794989624636099884에 FOLLOW_POSE_SCAN_TIME_MISMATCH 예외가 발생했다.
마지막 지도 위치 원본 시각을 기준으로 한 당시 나이는 약0.556초로0.8초 제한 이내다.
따라서 전체 입력 나이 검사와 두 입력의 상호 시각 차이 검사는 구별해야 한다.
예외를 일으킨 정확한 scan/map 쌍은 원래 exception 로그에 저장되지 않아 임의로 수치를 채우지 않았다.

지도 위치 수신 trace에는 종료 직전에도 동일한 오래된 원본 시각을 가진 메시지가 있고,
종료 처리 중에는 새 원본 시각1794989624810176578의 지도 위치가 다시 들어왔다.
이는 영구적인 지도 위치 소실이 아니라 이번 구간에서 지도 위치 갱신이 뒤처졌다는 증거다.

모터 기록:
- 예외 직전/직후 telemetry_isolated=true, fault=null.
- feedback count는 계속 증가했고 마지막 양쪽0속도 확인 및 post_stop_faults=[]다.
- 세션 종료 요청 뒤 disarm됐다. 모터 RESPONSE_TIMEOUT이 이번 종료를 먼저 일으킨 기록이 아니다.

## 두 버전에 실제 같은 입력을 넣은 재생

GitHub 원본 build_scene과 현재 build_scene에 같은7개 시각 조합을 넣어 각각 실행했다.
0.4초 차이는 허용되고0.400000001초부터 두 버전 모두 같은 오류를 발생시켰다.
모든 입력의 나이는0.8초 미만인 상태다.
오류가 recoverable_reference_loss로 분류되는지도 원본/현재 모두False임을 확인했다.
코드 분기와 현재 로그의 종료 이유가 일치한다.
이는 ROS·DDS·모터를 실행한 시험이 아니라 실제 순수 함수에 대한 오프라인 재생이다.

## 아직 확정하지 못한 원인

확정한 직접 원인은 지도 위치와 스캔의 시각 불일치 및 이 오류를 즉시 종료로 처리하는 기존 정책이다.
왜 그 구간 지도 위치 갱신이 늦었는지는 이 로그만으로 다음을 분리하지 못한다:

- EKF 또는 TF 갱신 지연.
- Bridge가 오래된 EKF/TF 원본 시각을 보존해 반복 발행.
- Bridge 또는 평가 실행기의 DDS 전달/콜백 처리 지연.

Bridge 소스는 EKF와 TF 중 더 오래된 원본 시각을 odom_map header로 보존한다.
따라서 새 메시지를 받았다는 사실만으로 지도 위치 원본 시각이 새롭다고 볼 수 없다.

정량1의 spin(.01초) + drain(0초) 호출 구조도 원본과 현재가 같다.
정량3에는 persistent executor 수정을 적용했지만 정량1 운영 실행기에는 아직 적용하지 않았다.
대기 큐 starvation 가능성이 남아 있지만, 이번 지도 위치 지연의 단독 원인이라고 단정할 증거는 없다.
오늘 미무장 모터의 DDS 발행0.726초 지연과 이번 지도 위치 지연은 별개 기록으로 취급한다.

## 결론 및 변경 범위

1. 성공 백업과 정량1 제어 원본이 달라져 검사 조건이 바뀐 것이 아니다.
2. 원본에도 있었던0.4초 불일치 종료 분기를 이번 실물 입력이 밟았다.
3. 이전의 "실행 명령이 맞다"는 안내는 해당 정량1 운영 실행기에 수신 수정이 적용됐다는 보증이 아니었다.
4. 이번 감사에서는 운영 코드·시각 제한·복구 정책을 변경하지 않았다.
5. 후속 수정 시 목표1의 map pose/scan 동시 시각 기록과 콜백 처리부터 검증해야 한다.
   단순히0.4초를 늘리거나 오류를 성공으로 바꾸면 동작 원인을 해결한 것이 아니다.

파일별 근거:
- all_backup_files.json: GitHub 동일성 증명과 모든1,347개 파일 비교
- goal1_profile_files.json: 원본324개 프로필 파일 비교
- active_driver_vs_success.json: 활성 드라이버 사본 차이
- time_guard_replay.json: 원본/현재7개 입력 재생
- latest_failure_evidence.json: 원문에서 추출한 마지막 제어·지도 위치·모터 결과
- callback_dispatch_comparison.json: 목표1/목표3 콜백 호출 비교
- compare_success_backup.py: 비교·재생 스크립트
