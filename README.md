# RMF + VDA5050 Simulator / Direction Arbiter

Open-RMF 기반의 실제 운영 흐름을 PC에서 재현하기 위한 통합 테스트베드입니다.

현재 주 작업 대상은 과거 `rmf_lab_v23.xx` 독립 실험 시뮬레이터가 아니라, 다음 End-to-End 경로입니다.

```text
Task / Order
  -> Direction Arbiter Task Gate (:8200)
  -> RMF API / Dispatcher (:8100)
  -> RMF Planner / Schedule / Negotiation
  -> VDA5050 Fleet Adapter
  -> MQTT (:1883)
  -> VDA5050 Robot Simulator
  -> VDA5050 State feedback
  -> Fleet Adapter / RMF task completion
```

## 현재 목표

FAB의 우회로 없는 1차선 양방향 통로에서 반대 방향 로봇이 동시에 RMF planning/negotiation에 들어가며 발생하는 교착, timeout, `endpoint_exchange_without_buffer` 류 문제를 줄이기 위해 **RMF planning 전에 방향 admission을 결정**합니다.

핵심 원칙:

- RMF Core/Planner/Negotiation 자체는 우선 유지
- 반대 방향 task를 동시에 RMF에 submit하지 않음
- Direction Arbiter가 `GRANT / WAIT / BLOCKED`를 결정
- 대기는 corridor mainline 내부가 아니라 Holding Bay / parking / upstream safe node에서 수행
- Direction switch는 active robot과 corridor occupancy가 실제로 clear된 뒤에만 허용
- telemetry stale/robot fault 시 fail-safe하게 반대 방향 release 금지
- TOP / CENTER / BOTTOM 및 다중 block으로 확장 가능한 구조 유지

## 주요 폴더

```text
traffic_control/                         Direction Arbiter core + task gate
config/                                  Corridor / P4 block configuration
tests/                                   Arbiter, occupancy, dynamic insert, P4 config tests
scripts/                                 Arbiter 실행 / T4 dispatch / workspace 검증
docs/                                    실행 가이드, 설계, 구현 지시서
docs/history/                            과거 rmf_lab v23 계열 patch history
rmf_dev_tool-main/vda5050_robot_simulator/ VDA5050 Robot Simulator
rmf_dev_tool-main/vda5050_gui/           GUI/로그 도구
rmf_platform-main/                       Docker/RMF runtime config 및 P4 adapter map/config
archive/                                 복원본 및 중간 checkpoint 원본 보존
```

## T1 ~ T4 실행 개념

### T1 — RMF Core

RMF API/Dispatcher, Traffic Schedule, Planner/Negotiation 등 기반 서비스를 구동합니다.

### T2 — VDA5050 Fleet Adapter

RMF navigation/task 요청을 VDA5050 Order로 변환하고, MQTT State를 다시 RMF에 반영합니다.

### T3 — VDA5050 Robot Simulator

실물 AMR 대신 VDA5050 Order를 받아 이동하고 State를 발행합니다.

### T4 — Task dispatch

직접 RMF `:8100`으로 보내는 대신 Direction Arbiter `:8200`을 통과시키는 것이 현재 PoC의 목표입니다.

```bash
./scripts/run_direction_arbiter.sh config/corridor_blocks_p4.yaml
./scripts/t4_dispatch_via_arbiter.sh AGV_A1 <GOAL_NODE>
```

## Direction Arbiter

주요 모듈:

- `traffic_control/models.py`
- `traffic_control/direction_arbiter.py`
- `traffic_control/corridor_registry.py`
- `traffic_control/robot_tracker.py`
- `traffic_control/task_gate.py`

동작 개념:

```text
new task
  -> latest robot telemetry 확인
  -> route / controlled block 분류
  -> block direction + destination HB reservation
  -> GRANT: RMF API로 forward
  -> WAIT : local queue 유지
```

반대 방향 waiter가 생기면 현재 batch를 무한히 연장하지 않는 batch-close 정책을 사용합니다.

## P4 runtime files

2026-09-16 집 환경 재현을 위해 다음 파일이 포함되어 있습니다.

- `rmf_platform-main/docker-compose.p4.yml`
- `rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_edit_node_add.yaml`
- `rmf_platform-main/src/rmf_vda5050_fleet_adapter/config/p4_edit_before.yaml`
- `rmf_dev_tool-main/vda5050_robot_simulator/p4_scenario.yaml`
- `config/corridor_blocks_p4.yaml`

## 현재 검증 상태 — 2026-09-16

이 저장소로 정리하면서 최신 Direction Arbiter + P4 patch를 overlay한 뒤 다음 검증을 수행했습니다.

```text
python3 -m pytest -q tests
34 passed
```

현재 알려진 runtime 이슈:

- RMF API의 `/tasks/robot_task`는 인증/권한 검사를 사용함
- Task Gate에서 RMF `:8100`으로 forward할 때 올바른 Bearer/JWT 인증 전달이 아직 최종 정리 중
- 최근 실제 실행에서 T4 submit은 `401 Unauthorized` 확인 단계까지 진행
- 따라서 현재 병목은 Direction Arbiter state machine 자체보다 Task Gate -> RMF API authentication 연결 쪽

실제 Ubuntu/ROS2 Jazzy/Docker 환경의 T1~T4 전체 runtime은 대상 PC에서 다시 검증해야 합니다.

## 문서

- `docs/direction_arbiter_guide.md` — 현재 Arbiter 사용 가이드
- `docs/simulation_run_guide.md` — 시뮬레이션 실행 가이드
- `docs/specs/directional_block_arbiter_implementation_spec.md` — 상세 구현 지시서
- `docs/specs/direction_arbiter_segmented_corridor_implementation_spec.md` — segmented block 설계
- `docs/specs/corridor_manager_spec.md` — 이전 Corridor Manager 설계
- `docs/PROJECT_HISTORY.md` — 지금까지의 작업 흐름과 버전 계보

## Archive 정책

`archive/`에는 현재 main runtime과 섞으면 안 되는 복원본/중간산출물을 그대로 보관합니다.

특히 2026-09-15 `vda5050_t2_reconstructed`는 대화에서 확보한 소스 조각을 이용한 **compatibility reconstruction**이며, 실제 회사 원본 소스와 byte-for-byte 동일하다고 간주하면 안 됩니다.

## 주의

- `.env`, token, JWT secret, 로그/빌드 산출물은 커밋하지 않습니다.
- `build/`, `install/`, `log/`, `.venv/`, `results/`는 `.gitignore` 대상입니다.
- 실제 운영환경 적용 전에는 real map, robot footprint, HB가 진짜 off-mainline인지, telemetry freshness 및 fail-safe 조건을 반드시 검증해야 합니다.
