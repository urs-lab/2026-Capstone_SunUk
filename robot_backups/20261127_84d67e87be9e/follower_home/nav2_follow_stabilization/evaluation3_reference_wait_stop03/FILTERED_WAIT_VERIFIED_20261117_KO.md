# FILTERED_STALE 검증 및 수정

원본 로그에서 EKF source stamp 공백0.864101451초, 수신 공백0.845713177초를 확인했다.
오류 발생 시 오래된 EKF 데이터 age0.807770967초. 모터 fault는 없었고 양쪽 정지는 확인됐다.
참조 입력 대기 중 local_zero_hold가 core.scene을 호출하여 FILTERED_STALE를 다시 던지는 경로였다.
EKF는 종료 뒤 새 메시지가 수신된 기록이 있다. 내부 발행/DDS/콜백 정체의 정확한 원인은 미확정이다.

수정은 평가3 3cm 운영 프로그램에만 적용했다. 드라이버, 정책 파일, 계획10cm·정지3cm는 그대로다.
EKF의 source/receive age가 이동용0.6초를 넘으면 속도0으로 무기한 대기한다.
그동안 정상 raw/IMU/scan의 원본 시각으로 STOP만 보내며 이전 EKF/지도 위치를 생성하거나 재스탬프하지 않는다.
새 EKF·스캔 정렬과 최소3개 신규 표본의1초 안정 및 모터 정지 피드백을 확인한 뒤
같은arm·중단 단계에서0속도 ramp부터 재개한다. 대기 중 운영 타이머를 멈추고 겹친 대기는 중복 합산하지 않는다.

미래/잘못된 EKF, raw/IMU/scan 이상, 실제 모터 fault, 재시작/비무장, 기기 watchdog은 유지한다.
이 검증은 오프라인이며 실제 로봇 주행 완료나 통신 문제의 영구 제거를 보증하지 않는다.
세부 검사 수·해시는 filtered_wait_implementation_report_20261117.json을 참조한다.

재실행은 기존 명령 그대로:
bash ~/nav2_follow_stabilization/success_tf_recovery/start_evaluation3_wait_stop03.bash

이전 시험이 종료된 뒤 로봇을 재배치했다면 선두Bridge 경로 초기화·양쪽 초기 위치 지정·기준선 재저장을 한다.
드라이버와 센서/AMCL/Bridge의 재시작은 이번 코드 수정 자체에는 필요하지 않다.
GitHub의 기존 성공 스냅샷과 보고서 전송 파일은 자동으로 변경하지 않았다.
