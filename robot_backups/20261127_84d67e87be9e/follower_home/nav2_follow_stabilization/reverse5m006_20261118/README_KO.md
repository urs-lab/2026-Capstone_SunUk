# 선두/후발 개별 5m 후진, 명령 속도 0.06m/s

성공 평가1~3 소스와 실행 경로는 수정하지 않음. 한 대씩 실행.
Nav2/gate, 추종, 평가3, teleop, 다른 거리 운영자는 모두 종료. 뒤쪽 경로 확보.
명령 속도 -0.06m/s, 출발 시점 차체 방향으로 투영한 원시 오뎀 5m에서 정지.
회전/횡방향 드리프트 검사는 유지. 역방향 실측 보정은 미검증이므로 거리 계수 1.
장애물 거리 정지는 이 명시적 거리 이동에서 사용하지 않음. 실물 관찰 및 전원 차단 수단 필요.
주행 종료 또는 Ctrl+C 시 속도0/비무장 처리, 새 측정 피드백이 없으면 정지 확인 성공으로 기록하지 않음.

선두 기존 모터·센서·EKF 유지. 후발에서 최초 복사:
`bash ~/nav2_follow_stabilization/reverse5m006_20261118/copy_leader.bash`
선두 실행:
`bash ~/leader_diagnostics/reverse5m006_20261118/reverse_leader.bash`

후발 기존 드라이버 종료, 센서·EKF 유지:
`bash ~/nav2_follow_stabilization/reverse5m006_20261118/start_follower_driver.bash`
별도 후발 터미널:
`bash ~/nav2_follow_stabilization/reverse5m006_20261118/reverse_follower.bash`
각 운영자는 START + Enter 후 해당 로봇만 후진. 두 로봇 동시 실행 불필요.
후발 드라이버는 전용 finite distance flag에서만 후진 허용, 최대 0.06m/s·10RPM,
회전/전진/초과 속도 차단, 5.5m 드라이버 거리 상한과 140초 상한 유지.
선두 최대140초, 각 운영자는5m에서 먼저종료. 계산상5/0.06=83.33초.

검증: 후발 실제 계약/포트+MemoryTransport 및 운영자 fake ROS 7건, 선두 fake ROS 3건 통과.
후발 MemoryTransport 전체 5.0038m 후진 81.93초·비무장0RPM 확인(정수 RPM 모델).
실물 후진 주행은 수행하지 않음. 실제 속도/거리는 구동계·오뎀 오차 및 정지 지연 영향 있음.
평가 복귀 전 후진 운영자와 후진 전용 드라이버 종료 후 평가용 드라이버 재실행.
