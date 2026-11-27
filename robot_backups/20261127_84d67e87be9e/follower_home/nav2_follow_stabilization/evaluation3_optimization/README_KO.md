# 평가 3 계산 최적화 후보

기준: final_robot/target_3/new_map_20261107의 보관 성공 코드.
원본 268개 Python 파일의 SHA256을 baseline_manifest.json에 기록하고 별도 사본을 만들었다.
보관 성공 코드와 현재 평가 2·일반 추종 코드에는 쓰지 않았다. 실제 모터를 구동하지 않았다.

확인한 지연 기록:
follower_diagnostics/logs/local_avoidance_trial_1794032200970674450.jsonl
실제 회피 planning_input 1794032231640511704부터 checked_local_plan
1794032271892285248까지 40.251773544초. 이는 기록상 경과 시간이며 순수 CPU 시간으로 단정하지 않는다.
같은 기록의 첫 planning_input은 사전 입력으로 장애물 군집이 없어 계산 비교 대상과 구분했다.

변경은 continuous_avoidance_candidate.py의 모의 주행 안 2차원 거리 계산 한 군데다.
짧은 벡터마다 NumPy 배열·범용 norm을 호출하던 부분을 math.hypot으로 변경했다.
12cm 목표 전환 경계 ±1e-12 부근은 원래 NumPy 계산으로 판정해 경계 결정을 보존한다.
모든 장애물 점, 전체 경로 검사, 모의 주행 0.05초 간격, 후보 순서와 개수,
20cm 계획 여유·10cm 정지 한도, 60초 계산 예산, 실시간 제어·구동 코드는 유지했다.

조회/모의 검증:

```bash
cd ~/nav2_follow_stabilization/evaluation3_optimization
python3 -B -m unittest test_planning_equivalence
python3 -B benchmark_planning.py
python3 -B compare_full_search.py
```

benchmark_results.json은 고정 경로 재생 3회씩의 결과다. 두 기록 모두 결과 사전과
전체 주행 trace가 원본과 정확히 같았다. 중앙값 기준 4.179→2.857초(약31.6% 감소),
4.020→2.855초(약29.0% 감소)였다. 이 장치의 오프라인 수치이며 라즈베리파이
전체 경로 계산이 같은 비율로 단축된다고 보증하지 않는다.
full_search_results.json은 후보 선택·정적 차체 검사까지 포함한 전체 탐색 비교다.
현재 주행 버전에는 자동 반영하지 않았다. 실행 명령과 입력 기준선을 섞지 않도록
별도 후보에서만 보존한다. 거리 한도를 줄이는 변경은 수행하지 않았다.

## 적용 및 실행

현재 센서·AMCL·Bridge·중계기와 연결하는 실행 스크립트를 추가했다.
정량평가 3은 Nav2 gate로 선두를 제어하는 일반 추종 시험과 별개다.
선두 Nav2/gate·모든 teleop·평가2 거리 시험·후발 일반 추종 프로그램은 종료한다.
센서·지도·AMCL·Bridge·중계기는 유지한다. 모터 드라이버는 아래 전용 모드로 교체한다.

후발 터미널 1: 기존 후발 모터 드라이버를 Ctrl+C로 종료한 뒤:

```bash
bash ~/nav2_follow_stabilization/evaluation3_optimization/start_follower_driver.bash
```

현재 장치 상태·엔코더 오류 검사가 있는 smooth010 모터 드라이버를
10RPM, continuous20, 360초로 실행한다. 일반 추종의 15RPM·총시간 무제한 모드와 다르다.
드라이버 재시작 후 후발 오뎀 원점이 바뀌므로 실제 후발 위치·방향을 RViz에서 재설정한다.
두 로봇을 정지 상태로 정렬하고 선두 Bridge 경로 기록도 새 시작 배치에 맞춘다.

후발 다른 터미널: 조회 전용 현재 지도 기준선 저장:

```bash
bash ~/nav2_follow_stabilization/evaluation3_optimization/capture_reference.bash
```

기준선 파일은 references/에 새 이름으로 저장한다. 성공한 파일 경로만
latest_reference_path.txt에 기록한다. 기존 보관 기준선을 덮어쓰지 않는다.
시작 배치를 바꾸거나 오뎀을 재시작하면 다시 캡처한다.

후발 시험 실행:

```bash
bash ~/nav2_follow_stabilization/evaluation3_optimization/start_evaluation3.bash
```

새 기준선이 없으면 주행 프로그램을 실행하지 않는다. 기본 TF는 현재 쓰는
/follower/tf_static_internal이다. 구형 공개 TF 센서 구성을 사용한다면
CONVOY_TF_STATIC_TOPIC=/tf_static으로 명시해 실행한다.
입력 점검 후 START + Enter를 입력해야 주행한다. 장애물 발견 후 정지→계산→회피→재합류를 수행한다.
실측 안전 검증은 기존 제어 방식 그대로이며 계산 최적화가 실제 기체에서 검증된 것은 아니다.
이 스크립트를 이번 작업 중 실제로 실행하거나 로봇을 arm하지 않았다.

최종 검증: 평가3 회귀 93개, 경계·동등성 5개, 실행 스크립트 4개,
현재 드라이버 continuous 계약 4개 통과. 실제 기체 시험은 수행하지 않았다.
