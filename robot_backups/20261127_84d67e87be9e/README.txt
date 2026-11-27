ROS2 선두·후발 졸업작품 소스 및 논문 인수인계 백업

사용자 요청 기준일2026-11-27. 실제수집시각은 metadata/COLLECTION_PROVENANCE.json 참조.
leader_home/: 선두에서SSH로직접수집한소스(누락root포함메타데이터보존).
follower_home/: 현재후발작업공간의관련소스·설정·지도.
paper_handoff/: 새GPT논문작성용인수인계·실행경로·실험로그색인·빈실측표.
evidence/historical_20261113/: 과거자료,최종시연측정치로대체하지않음.
MANIFEST.json / verify_snapshot.py: 모든저장파일의SHA256·크기·권한검증.

정량1은선두Nav2+후발실측경로추종,정량2는선두3m전진3회,정량3은후발회피→재합류→추종.
정량3 원본과선두속도범위검사해제실험은서로다른실행본이며결과를혼합하지않음.
성공확인대화·코드설정·원본로그·외부실측·모의시험은각각구분.
최종채점표/시연영상/시행별실측표가없으므로이백업은정량목표모두합격을주장하지않음.

소스보존만수행. ROS/모터실행·기존파일자동복원·시스템설정변경하지않음.
새장치에는ROS2 Humble와외부의존성·장치권한·colcon build가필요.
LiDAR trace 실행파일은원본C++/CMake소스로재빌드필요. build/install은백업대상이아님.
AMCL용TF2 overlay .so는관련폴더에보존했으며ARM64·원래패키지환경과호환성확인필요.
관련소스폴더의과거실험도보존했으므로실행은 paper_handoff/03_RUNBOOK_KO.txt의진입점으로구분.
일부예전test가 logs/의대형fixture를참조할수있으며전체회귀시험의재현을자동보장하지않음.
개인홈전체·SSH키·인증토큰·대용량bag는저장하지않음. 소스내기존주소/장치경로는그대로보존.
프로젝트저작권과기존제3자LICENSE/NOTICE를유지하며임의의새오픈소스라이선스를부여하지않음.
GitHub 업로드완료는상위도구의PUBLISH_STATUS.json또는원격commit검증으로확인할것.
