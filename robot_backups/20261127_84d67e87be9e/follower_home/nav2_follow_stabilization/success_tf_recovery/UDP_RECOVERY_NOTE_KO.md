# 2026-11-17 AMCL TF 정체에 대한 추가 복구 진입점

관측: raw/filtered/scan 및 진단 노드의 odom TF는 갱신되지만 AMCL 내부 odom TF 시각은
1794892564.974851에 머물렀다. 초기 위치는 여러 번 수신했고 지도가 수신된 로그도 있다.
성공 백업과 AMCL YAML/localization launch/stack launch는 동일했다.
정확한 DDS 내부 원인은 미확정이다. UDP 전용 실행은 복구 시도이며 실물 검증은 필요하다.

기존 start_localization.bash와 launch/YAML은 변경하지 않았다.
선두에서 사용한 원본 UDP 프로필을 복사하고 이 추가 wrapper에서만 두 프로필 환경변수를 설정한다.
후발2의 기존 통합 launch를 Ctrl+C로 종료하고 모든 하위 프로세스 종료를 확인한 뒤:

bash ~/nav2_follow_stabilization/success_tf_recovery/start_localization_udp.bash

센서가 올라온 뒤 RViz에서 /follower/initialpose로 실제 초기 위치를 지정하고 public TF 진단을 수행한다.
통합 스택을 중복 실행하지 않는다. 모터 드라이버와 relay는 유지하며 Goal/추종은 진단 완료 전 시작하지 않는다.
원래 실행으로 돌아가려면 이 스택을 종료하고 기존 start_localization.bash를 실행한다.
원격 GitHub 성공 백업에는 자동 반영하지 않았다.
