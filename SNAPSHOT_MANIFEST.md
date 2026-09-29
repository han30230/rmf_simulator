# Snapshot Manifest

이 저장소는 집과 회사 환경에서 재현한 RMF/VDA5050 작업본과 P4 Passing Bay PoC를 하나로 합친 실행 스냅샷이다.

## 병합 순서

1. `vda5050_t2_reconstructed_2026-09-15`
2. `rmf_simulation_workspace_direction_arbiter_2026-09-16`
3. `rmf_p4_home_runtime_patch_2026-09-16`
4. `rmf_direction_arbiter_api_auth_patch_2026-09-16`
5. 실행 진입점이 포함된 3515줄 VDA5050 GUI 복구본
6. RMF API `0.0.0.0:8100` 로컬 설정
7. P4 Single Passing Bay PoC와 2106 무정지 조기 해제
8. 새 PC용 setup/start/stop 자동화와 portable MQTT 구성

## 검증

- Python 단위·구성·상태기계 테스트: 60개 통과
- `compileall`: 전체 Python 문법 검사
- P4 Map/Fleet/Scenario/Arbiter 설정 일치 테스트
- HTTP Bearer Token 전달 테스트
- GUI의 `SimulationTab`, `MainWindow`, `main()` 진입점 확인
- portable workspace 파일, repository-relative 경로, shell 문법 확인

## 포함하지 않는 항목

- `.venv` 및 Python 캐시
- Docker/ROS 빌드 산출물
- 실행 로그와 PID
- SQLite DB 및 API 캐시
- 실제 JWT, 비밀번호, `.env`
- Docker 이미지 자체

Docker 이미지 자체는 포함하지 않는다. `scripts/setup_workspace.sh`가 공식
Open-RMF 이미지를 준비하고, `scripts/start_p4_passing_bay.sh`가 필요한 경우
Mosquitto container를 시작한다.
