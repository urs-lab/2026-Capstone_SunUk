# 선두 단독 9m 후진, 최대 명령 속도 0.5m/s

성공 평가1~3 소스와 5m 후진 코드는 변경하지 않음.
Nav2/gate·추종·평가3·teleop·다른 거리 운영자 종료. 후발은 비무장으로 후진 경로 밖에 둠.
기존 선두 모터·센서·EKF launch를 종료한 뒤 함께 복사되는 leader_eval2_speed050.launch.py allow_motion:=true로 교체.
기존 평가2 백업과 동일한 max_rpm=57, 반경0.085m, 명령변환/피드백보정 유지. 중복 드라이버 금지.
9m는 매 실행 출발 원시 오뎀 기준, 역방향 실측 계수 미검증이므로1.0.
0.5m/s는 최대 명령값이며 가속0.2m/s²·감속0.3m/s²·도착 전 감속 유지.
시간상한42초, 15도방향/15cm횡이탈/센서신선도/단일명령주체/최종속도0확인 유지.
운전자 경로확인 모드: LiDAR 장애물 거리 정지는 사용하지 않음.
후방9m 및 제동 여유를 확보하고 관찰/전원차단수단 준비. 실물주행/정지거리/9m정확도는 미검증.

후발: bash ~/nav2_follow_stabilization/reverse9m050_20261118/copy_leader.bash
선두 모터 터미널 교체: ros2 launch ~/leader_diagnostics/reverse9m050_20261118/leader_eval2_speed050.launch.py allow_motion:=true
선두 별도 터미널: bash ~/leader_diagnostics/reverse9m050_20261118/reverse_leader.bash
START + Enter로 출발, Ctrl+C 중단.
모의 검사3건통과: 서로다른출발점9m후진·가속/감속·명령범위·다른발행자차단·센서손실/중단속도0.
