# 정량1 상태 전달과 속도 연계 조정 (별도 실행)

원래 정량1/2/3 및 모터/센서 소스 수정 없음. 후발 드라이버는 기존 정량1 15RPM용 유지.
기존 최고속도0.1m/s, 가감속, 목표 간격1.5m, motor .3초와 operator .8초, leader source .3초 검사 유지.

실측 로그1794996949081746918: 상태 생성 시 source age 평균.205초, 대략.12초 상태 생성 주기,
기존 중계 timer 최대.05초 부가대기. 선두 실제로그: Nav2 .12~.15명령에도 gate .04/0 반복.
지연 분포가 있어 드문 SOURCE_STALE까지 제거한다고 보장하지 않음. 신선도 기준을 늘리거나 시각을 덮어쓰지 않음.

변경:
- operator: 상태 전송 최소간격 .1→.04초, 디스크 로그는 기존10Hz. 원래 measurement/status시각 유지.
- follower relay: channel5 상태 callback에서 즉시 UDP전송, 그 상태 재송신캐시/timer 대기제거.
  다른channel/peer/port/wire/시간검증 유지. 원래 relay는 수정안함.
- leader gate: smooth 출발 저속상한 .04→.06. gap>=1.5m일때 선두상한 min(.1,후발요청+.02).
  gap<1.5m일때 기존속도연계 유지. 부족간격/역주행/실제fault/오래된입력 차단 유지, w/v 유지.
- leadercopy는 현재 da0ab05848da 소스5개 해시가 검토한 성공백업과 일치해야만 별도폴더에복사.

기존 정량1 추종, 후발 유니캐스트, 선두5 Nav2/gate를 종료. 모터·센서·Bridge 유지.
후발에서: bash ~/nav2_follow_stabilization/goal1_flow_20261118/copy_leader.bash
선두5: bash ~/leader_diagnostics/goal1_flow_20261118/start_leader_nav2.bash
후발 유니캐스트: bash ~/nav2_follow_stabilization/goal1_flow_20261118/start_unicast.bash
후발 추종: bash ~/nav2_follow_stabilization/goal1_flow_20261118/start_follow.bash
RViz 선두2D Goal지정. 단독Bridge 추가실행불필요.

모의 검사12건통과: gate조건/곡률/시간경계/기존원본일치/UDP원문유지/지연재현7건,
실제operator fakeROS 대기재개·실제fault종료·plan무명령·RPM15·가감속/최종정지5건.
실물속도향상/연속주행은 미검증. 이전복귀는 새3개종료후 기존선두5·relay·start_follow실행.
정량3는 선두5/추종종료, 기존정량3드라이버와 평가프로그램으로전환. 센서/Bridge는하나만유지.


첫 실측경로 대기 복구 수정:
실제로그1794998408728232453에서 첫 경로는1점이며 아직 유효2점경로를 수신하지 못한 상태.
선두참조 지연으로 recovery진입 후 fresh참조가 돌아오면 recovery가 nav.command를 직접 호출해
singleton INVALID_GEOMETRY_OR_FRAME를 STOP_MISSING_OR_INVALID_PATH로 최종종료하던 누락재현.
복구중에도 일반startup과같은 zero-follower bootstrap을 적용. 첫2점경로전에는 follower명령0유지.
leader/follower_map/scan 원래시각의1초안정·3개서로다른표본·정지피드백을 확인하고 기존선두저속기동대기로복귀.
path시각을 가짜로 채우지않음. 유효경로를한번수신한뒤의복구는 기존처럼path표본도요구.
회귀: 수정전singleton/staleleader2case동일오류재현, 수정후2case통과(첫경로없음계속0·나중2점후추종).
기존경로후대기재개·실제fault·raw손실·ramp재개4case통과. 실제기동은사용자확인필요.
이번수정은후발operator만수정. 선두코드재복사/센서·드라이버·중계기재시작불필요.


## 명령 전달 중 source expiry 대기 (정량1 전용 driver)
실제로그1794998687070034233의 첫 fault: INVALID_LOCAL_AVOIDANCE_ENVELOPE,
last_command_rejection=Stale/future/pre-arm source, feedback_fresh=true, device_status=0/0,
response/io error 없음. 종료 직전 source_age_after_compute .725초 표본 확인.
기존 source.8초 경계에서 전달 지연과 겹치는 상황을 MemoryTransport로재현.

새 driver/는 goal1_transport_20261118의 독립 복사본. 기존 driver/start파일은 수정안함.
--wait-expired-reference 명시시 extended15RPM local_avoidance에만 적용.
원래세션/프레임/헤더.3초/측정시각순서 및 송신시 source.8초를 모두통과해야
수신시 만료한 source만ZERO명령으로변환. nonzero로재개하지않고 active hold상태유지.
새로운fresh zero와 실제3개정지피드백으로만hold해제. operator는 상태flag를보고freshlocal zero를보내며,
기존1초안정·3개고유지도/참조/scan·정지피드백검사를통과후동일세션에서ramp재개.
operator는송신전source남은시간.15초미만이면미리zero대기. .8초TTL자체는변경하지않음.
미래시각/송신전에이미만료/다른epoch/prearm/과속/명령loss/피드백loss 차단유지.

이번수정적용: 기존후발드라이버Ctrl+C후
bash ~/nav2_follow_stabilization/goal1_flow_20261118/start_driver.bash
별도후발: bash ~/nav2_follow_stabilization/goal1_flow_20261118/start_follow.bash
선두5/센서/Bridge/중계기유지, 새RViz Goal지정. 정량3는기존정량3드라이버로교체.
모의검사20건통과: newdriver5·operator연동/firstpath/fault5·기존extended/source검사10.
실물재개확인은아직없음. 실제fault는자동clear/자동rearm하지않음.
