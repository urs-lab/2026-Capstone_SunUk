# 모터 드라이버 ROS 발행 정체 분리

관측: 비무장·0RPM·장치상태0에서 status publish가1.67697초 걸려 다음 모터 tick이
1.67864초 늦어졌다. 수신 버퍼에는20byte가 있었으나 기존0.3초 응답 deadline을 넘겨
RESPONSE_TIMEOUT이 발동했다. 이 기록만으로 모터 미응답·케이블 불량을 확정하지 않는다.

현재 start_follower_driver.bash에는 --isolated-telemetry를 추가했다.
raw/position/status 발행을 topic별 독립spawn 프로세스로 분리했다.
자식은 ROS 발행만 하며 직렬 포트·모터 명령·arm/disarm 인터페이스가 없다.
직렬 코어·position 검증·command/service는 원래 제어 프로세스에 남는다.
부모의 DDS raw/status/position publish 호출은 작은 메시지 직렬화와 mmap 최신슬롯 갱신으로 바뀐다.
부모 슬롯 lock은 nonblocking이다. 자식은 메시지 복사를 끝내고 lock을 해제한 뒤 DDS에 발행한다.
원본 시각을 유지하며0.3초보다 오래 대기한 발행 스냅샷은 버린다. 막힌발행 한 건의 전송을
취소할 수는 없으나 새 큐를 모두 재생하지 않고 최신 상태로 진행한다.

status 발행만 정체돼도 raw odom까지 멎지 않도록 프로세스를 각각 둔다.
발행자 이름은 follower_drive_telemetry_raw / position / status로 보인다.
모터 명령 구독자와 arm/disarm 서비스 주인은 기존 follower_drive_candidate다.
위치 계산 오류와 엔코더/장치 상태 검사는 동기 실행하여 fail-closed를 유지한다.
자식이 종료되면 다음 tick에서 TELEMETRY_PROCESS_FAILED로0속도·잠금한다.
종료 시 모터 shutdown/포트 close를 먼저 수행한 뒤 발행 전용 자식을 정리한다.

0.3초 command/feedback/response timeout은 변경하지 않았다.
PyDLL로GIL을 잡은1.7초 발행 지연을 실제 자식 프로세스에 주는 시험에서 부모의
메모리 모터 응답 처리·동일epoch 유지·0RPM·최신 발행만 전달을 확인했다.
전 프로세스/부모 스케줄링 지연이나 실제 직렬 응답 손실까지 해결했다고 주장하지 않는다.
ROS 메시지 serialize/deserialize도 실제 설치 Humble 라이브러리로 원본 stamp 보존을 확인했다.
모든 검사는 오프라인/메모리이며 실물 시리얼 포트나ROS 노드를 실행하지 않았다.

재실행:
1. 운영 프로그램·기존 후발 드라이버 종료. 두 드라이버를 함께 켜지 않는다.
2. bash ~/nav2_follow_stabilization/evaluation3_reference_wait_stop03/start_follower_driver.bash
3. fault=null / feedback_fresh=true / telemetry_isolated=true 확인.
4. 드라이버 재시작으로odom 원점이 바뀌므로 후발 초기 위치 확인 및 평가3 기준선 새 저장.
5. 기존 start_evaluation3_wait_stop03.bash로 실행.

기존 성공GitHub 스냅샷은 변경하지 않았다. 실제 로봇 시간/연결 재확인이 필요하다.
