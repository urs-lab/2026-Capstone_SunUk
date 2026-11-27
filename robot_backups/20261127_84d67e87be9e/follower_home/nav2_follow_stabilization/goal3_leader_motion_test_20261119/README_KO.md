# 정량3 선두 속도 허용범위 검사 해제 실험 (별도 실행)

기존 goal3_success_tf02524 및 정량1/2 소스는 수정하지 않음.
해제 대상은 LocalCore.scene의 LEADER_UNEXPECTED_MOTION 검사 한 가지:
선두 linear.x < -0.01 또는 >0.1 m/s, abs(angular.z)>0.1 rad/s일 때 종료하던 조건.
새 --leader-motion-range-test 인자를 명시한 전용 실행본에서만 해당 검사 해제.
기본값은 검사 활성. 유한값/프레임/세션/시각/센서/모터/장애물/지도연속성/합류 검사는 유지.
선두 모터 최고속도를 올리는 작업이 아니며, 후발 명령0.025m/s·10RPM·여유10cm/정지5cm·합류후2m 유지.
다른 이유의 대기/정지/종료까지 해제한 것이 아님.

ROS 노드: /goal3_leader_motion_range_test
같은 토픽에 명령하므로 기존 평가3 운영자·정량1추종·teleop·선두Nav2/gate는 종료하고 실행.
기존 평가3 드라이버, 센서/위치추정 통합, 양쪽Bridge와중계기는 유지.
양쪽 위치/방향을 맞추고 정지·후발비무장 상태에서:

bash ~/nav2_follow_stabilization/goal3_leader_motion_test_20261119/capture_reference.bash
bash ~/nav2_follow_stabilization/goal3_leader_motion_test_20261119/start_test.bash

START + Enter로 시작. 테스트도 원본과 동시에 실행하지 않음.
테스트 기준선·latest_reference_path·주행JSONL은 전용 폴더에 저장하여 기존기록 덮어쓰지 않음.
속도 초과가 관측되면 console [TEST]와 JSONL event=leader_motion_range_observed에
원래 source_ns, 실제 linear_x_mps/angular_z_radps, exceeded 목록을기록.
종료JSON에 leader_motion_range_check_enabled=false, leader_motion_range_test=true,
leader_motion_range_exceeded_samples 수치 포함. 속도값을 바꾸거나 가짜시각을 넣지 않음.

원본복귀: 새테스트종료후 기존 goal3_success_tf02524의 capture_reference.bash와
start_evaluation3_ekf_wait.bash 사용. 기존script는 그대로존재.
모의검사9건통과:원본3방향경계유지,옵트인해제,stale검사,비유한값검사,
실제속도진단,후발속도/장애물정지,독립nodeflag,참조대기재개,IMU오류.
실물주행은 수행하지 않았음. 범위를 넘는 입력 자체로는 종료하지 않는 실험임을 인지하고 관찰.


복사본 기동 의존성 보완:
원본 operator_status_process는 parents[3]/transport_driver에서 isolated_telemetry를 찾음.
별도 폴더구조에서는그경로가없어 ModuleNotFoundError발생. 성공본helper를새candidate폴더에동일바이트로복사,
operator_status_process의기존상위경로우회를제거하고같은폴더helper를사용하도록수정.
새프로세스 python -I -B에서 전체운영자/상태helper import확인.
실제spawn프로세스와LatestSlot으로상태바이트송수신·정상정리확인(ROS/DDS/모터없음).
기존평가3파일은해시로변경없음확인. 제어설정추가변경없음.
