# 오류 기록 기능 준비 완료

후보 follower_drive_node.py에 선택 옵션 --fault-snapshot-dir를 추가했다.
기존 파일은 follower_drive_node.before_fault_snapshot.py로 보존했다.
모터 설정·패킷·주행 계산·기본 실행 방식은 변경하지 않았다.

오류를 처리하는 제어 tick 이후, ROS 상태 발행 전에 메모리의 오류 증거를
복사한다. 실제 파일 쓰기는 별도 스레드에서 수행한다. 대기열은 1개이며
디스크가 느려도 제어 콜백은 파일 쓰기나 대기열을 기다리지 않는다.
파일 쓰기 실패는 보존하고 종료 때 출력한다. 저장 완료된 파일만 .json으로
표시한다. 프로세스 강제 종료나 전원 차단 시 저장을 보장하지 않는다.

기록 내용은 fault, 마지막 송신 RPM, 현재 피드백과 장치 상태,
기존 io_history의 tx/rx/feedback, 최근 tick 시간이다. 기존 24항목 순환
이력을 사용하므로 연속적인 실제 직렬 통신 캡처가 아니다. 이전 명령의
전체 이력이나 장치 실행·물리적 정지를 증명하지 않는다.

## 실제 장비에서 가능한 다음 단계: 주행 없이 대기

추종 실행 프로그램과 기존 후발 모터 드라이버를 Ctrl+C로 종료한다.
센서·지도·RViz는 이번 기록 준비 때문에 재시작할 필요가 없다.
아래 명령은 motion input 활성화 옵션이 없으므로 arm과 주행이 차단된
대기 모드이며, 제로 RPM/피드백 통신은 한다. 외부에서 arm을 요청하지 않는다.

```bash
source ~/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash
python3 -B ~/nav2_follow_stabilization/candidate/follower_diagnostics/follower_drive_node.py \
  --run-hardware \
  --ack-hardware-zero-rpm-tx \
  --fault-snapshot-dir ~/nav2_follow_stabilization/reports/motor_faults
```

이 대기는 움직일 때 발생하는 문제를 재현하거나 수리하는 단계가 아니다.
오류가 발생하면 motor_faults/motor_fault_*.json을 남긴다. 오류가 없으면
오류 파일도 생기지 않는다. 지상 Goal 주행은 재개하지 않는다.

## 검증

fault_snapshot_regression.log: 새 검사 6개를 포함한 관련 검사 100개 통과.
디스크 지연·쓰기 오류·대기열 포화·복사본 독립성·기본 미실행 옵션을 확인했다.
실제 모터나 시리얼 포트를 열지 않았다. 보호 백업 818개 파일도 변경이 없다.
