# 평가3 계획10cm / 정지3cm — 2026-11-17 추가 구성

기존 evaluation3_reference_wait(정지5cm)는 보존했다. 이 폴더는 그 버전을 복사해
운영 프로그램과 드라이버의 STOP_CLEARANCE_M만0.03m로 맞춘 별도 프로필이다.
계획 여유0.10m, 속도/RPM, source timeout, 모터 고장/엔코더 검사와 watchdog은 유지한다.
기준·스캔 정렬·모터 상태 메시지 지연의0속도 대기와 같은 세션 재개도 유지한다.
운영/드라이버 프로필은 CONTINUOUS10_STOP03_WAIT_V1로 맞춰 기존5cm 드라이버와 혼용을 거절한다.

실행 중인 평가3 운영 프로그램과 후발 드라이버를 종료한다. 센서/AMCL/Bridge/중계기는 유지한다.
후발 드라이버:
bash ~/nav2_follow_stabilization/evaluation3_reference_wait_stop03/start_follower_driver.bash

드라이버 재시작 뒤 실제 초기 위치/방향을 지정하고 양쪽 정지·후발 비무장 상태에서:
bash ~/nav2_follow_stabilization/evaluation3_reference_wait_stop03/capture_reference.bash

REFERENCE_JSON 성공 뒤:
bash ~/nav2_follow_stabilization/success_tf_recovery/start_evaluation3_wait_stop03.bash

STOP_CLEARANCE는 LiDAR 원거리 자체가 아니라 차체 외곽·자세·불확실성 및 예상 이동을 반영한 값이다.
기록된 정지 장면의0.046418m는 새3cm 기준을 통과하는 것을 오프라인 제어 재생으로 확인했다.
이는 전체 실물 코스의 완료를 보증하지 않는다.

문제 스캔1794895132516286000에서 원거리0.340m/0.337m의 인접2점이 나타났다.
그 전후 약0.5초 범위의 각5개 스캔에서는 차체 모델13cm 이내 점이 없었다.
실물 박스와20cm 떨어져 보였다는 사용자 관측과 함께 볼 때 일시 반사/이상점 가능성이 있다.
그러나 실제 점의 물체 정체는 미확정이며 외부 점을 차체 내부 점으로 재분류하거나 자동 삭제하지 않았다.
원시 점·차체모델·모션을 분석한 근거는 clearance_anomaly_1794895061004562056.json이다.

GitHub 기존 성공 백업은 변경하지 않았다. 현재 새 프로필의 실물 확인은 아직 필요하다.
