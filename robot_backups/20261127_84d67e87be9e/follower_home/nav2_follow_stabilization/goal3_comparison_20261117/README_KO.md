# 성공 GitHub 구성과 현재 실행 비교

기준 커밋: wpdhks1029/capstone_goal3@7df1f0f69b947196e87db621a353aa3afc99cf16.
GitHub main도 이커밋임을 확인했고 원격manifest와 로컬캐시가 일치하며1617개파일 해시검증을 통과했다.

현재 유지: 계획10cm·정지3cm, 무기한 신호/EKF/모터상태 대기 및 회복 재개,
거리 위반 즉시0속도 후 새스캔 재관측, 실제 모터/센서 fault 보호.

변경점:
- 운영 프로그램: 참조대기에 EKF/모터상태/거리재관측 대기를 추가.
- 정책: 정지5cm→3cm, 운영/드라이버 프로필 동시 변경.
- 드라이버: 직렬Core/position계산/하드웨어YAML은 성공본과 같음. ROS발행만3개 자식으로 분리.
- 위치추정 launch/AMCL YAML/Bridge/중계기 소스는 성공본과 동일.
- 실행환경: 추가 UDP-only wrapper 때문에 EKF/AMCL/Bridge에 XML변수가 들어가 있음.

실제15초 관측:
모터 fault=null, feedback_fresh=true. raw/position 수신. filtered297개 및odom TF297개 최신age0.039초.
LiDAR113개수신. AMCL active이나 map→follower/odom TF는 수신되지 않음.
Bridge는 1794899726.354363의 공통시각을 계속조회하며 TFcache가 앞으로진행해 연결실패.
이에 지도오뎀 발행이 중단되고 RViz 후발표시가 사라진다.
모터 발행이 모두 멎어서 EKF가 사라졌다는 가설은 이 관측과 맞지 않는다.
정확히 AMCL 내부 어느콜백/통신이 멎었는지는 아직확정하지 않았다.

제한적인 되돌림:
후발2 기존통합스택만 종료하고 다음wrapper로 다시 실행한다.
bash ~/nav2_follow_stabilization/success_tf_recovery/start_localization_success_env.bash

이는 성공본 publicTF launch를 그대로 실행하고 추가한XML환경2개만 제거한다.
기존 UDP-only 실행본도 보존했다. 직렬/속도/거리/대기 설정은 변경하지 않는다.
초기위치를 다시 지정한뒤 audit_live.bash를15초 실행하여 원격갱신 여부를 확인한다.
복구 확인 전에는 주행 성공이나 원인 확정을 주장하지 않는다.
