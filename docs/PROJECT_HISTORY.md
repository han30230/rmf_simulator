# RMF Simulator Project History

## 1. RMF Traffic Lab 단계

초기 작업은 Stock Open-RMF의 좁은 1차선 양방향 통로 문제를 재현하고, Planner/Schedule/Negotiation의 동작을 정량 비교하는 독립 `rmf_lab_v23.xx` 시뮬레이터에서 시작했다.

대표 이슈:

- `endpoint_exchange_without_buffer`
- negotiation timeout
- `NO_PHYSICAL_ESCAPE`
- 2v1 / 2v2 / 3대 이상 동적 투입 교착
- 좁은 non-passing corridor에서 반대 방향 trajectory가 동시에 Schedule에 존재하는 문제

실험 방향에는 A* policy penalty, schedule soft cost, negotiation evaluator, reservation, CBS/ECBS/LaCAM/MAPF 검토가 포함됐다.

## 2. CBS_ROADMAP PoC — v23.20

`rmf_lab_v23_20_cbs_roadmap_poc.patch`

- Roadmap 기반 CBS PoC
- vertex/edge conflict 처리
- WAIT action 및 topology distance heuristic
- single-lane bidirectional / passing-bay 계열 검증
- 이후 production RMF 경로의 직접 대체보다는 비교/연구용으로 분리

## 3. Direction Epoch Corridor Manager — v23.21

`rmf_lab_v23_20_to_v23_21_corridor_manager.patch`

핵심 아이디어:

> 반대 방향 로봇을 동시에 RMF negotiation에 넣지 않는다.

- `1 corridor = 1 directional resource`
- epoch 시작 시 batch 고정
- 기본 batch max 3
- 현재 batch가 corridor를 모두 EXIT한 뒤 반대 방향 전환
- 신규 로봇이 현재 epoch에 무한히 끼어들지 않도록 제어

## 4. General Corridor Manager — v23.22

`rmf_lab_v23_21_3_to_v23_22_general_corridor_manager.patch`

- 특정 2v2/3v3 시나리오 하드코딩 제거
- 임의 robot 수 / 방향 비율 / request time 처리
- `--corridor-id`로 관리 resource 선택
- graph cut 기반 route direction classification
- dynamic insertion time을 bounded Stock-RMF epoch로 검증

## 5. Segmented Corridor Manager — v23.23

`rmf_lab_v23_22_to_v23_23_segmented_corridor_manager.patch`

- 긴 corridor를 C1/C2/C3... block으로 분할
- Global Direction Epoch + Block Pipeline
- 같은 방향 robot은 서로 다른 block에서 pipeline 진행 가능
- 반대 방향은 외부 parking/refuge에서 WAIT
- holding boundary 단위 단계적 release
- 마지막 block을 벗어난 뒤 실제 exit clearance를 확인하고 방향 전환

## 6. Corridor UX / Safe Spawn — v23.24

`rmf_lab_v23_23_to_v23_24_corridor_ux_and_spawn_fix.patch`

- Whole / Segmented / Legacy 시나리오 UX 정리
- user-added robot이 managed gate/mainline에 잘못 spawn되는 문제 수정
- 외부 parking/refuge를 우선 선택
- bounded runner 실행 중 일반 robot 추가가 즉시 반영되지 않는 점을 명시

당시 회귀 테스트 기록:

- 175 passed
- 4 skipped
- 109 subtests passed

## 7. 실제 RMF + VDA5050 통합 방향으로 전환 — 2026-09-14~15

과거 `rmf_lab_v23.xx`만으로는 실제 Fleet Adapter / Robot State / Task lifecycle을 완전히 재현하지 못하므로, 작업 대상을 실제 구조에 가까운 workspace로 전환했다.

```text
rmf_simulation_workspace/
├── rmf_platform-main/
└── rmf_dev_tool-main/
    └── vda5050_robot_simulator/
```

목표 E2E:

```text
RMF Core
 -> VDA5050 Fleet Adapter
 -> MQTT
 -> VDA5050 Robot Simulator
 -> State feedback
 -> RMF task completion
```

T1~T4 실행 순서를 기준으로 집/회사에서 동일하게 재현하는 작업을 진행했다.

## 8. VDA5050 T2 복원 — 2026-09-15

대화에서 확보한 source fragment와 공개 구조를 참고해 `vda5050_fleet_adapter` 호환 복원본을 만들었다.

주의:

- 회사 원본의 byte-for-byte 복구를 주장하지 않는다.
- advanced charging, commission manager, custom RMF patch 등은 원본과 다를 수 있다.
- 원본 확보 시 reconstructed 파일보다 원본을 우선 병합해야 한다.

## 9. Pre-RMF Direction Arbiter — 2026-09-16

현재 주 개발 방향.

기존:

```text
Task -> RMF Planner/Schedule/Negotiation -> Fleet Adapter -> Robot
```

변경 PoC:

```text
Task
 -> Direction Arbiter Task Gate (:8200)
 -> GRANT 된 task만 RMF API (:8100)
 -> Fleet Adapter
 -> MQTT (:1883)
 -> Robot Simulator
```

핵심 invariant:

1. 동일 managed block에 opposite direction 동시 admission 금지
2. direction switch는 실제 active robot/occupancy clear 이후에만 수행
3. Arbiter 때문에 robot을 corridor mainline 내부에서 강제 WAIT시키지 않음
4. inside robot telemetry stale이면 fail-safe BLOCK/FAULT
5. opposite waiter가 생기면 current batch를 닫아 starvation 방지

## 10. P4 적용

실제 P4형 TOP/CENTER/BOTTOM corridor와 Holding Bay/parking 구조를 대상으로 configuration을 확장하고 있다.

2026-09-16 기준 최신 patch에는:

- P4 nav/config
- 복수 robot simulator 설정
- Direction Arbiter P4 block configuration
- 집 환경 T1~T4 재현용 compose/script

이 포함되어 있다.

## 11. 현재 상태 — 2026-09-16

대화/라이브러리에서 확보한 최신 Direction Arbiter base + P4 patches를 합친 스냅샷에서:

```text
python3 -m pytest -q tests
34 passed
```

최근 실제 runtime에서는 Fleet Adapter와 simulated robot 등록/State 수신 단계까지 진행했다.

현재 남은 주요 blocker는 Task Gate가 RMF API `/tasks/robot_task`로 forward할 때의 authentication/authorization 연결이며, 최근 확인 상태는 `401 Unauthorized`이다.

다음 검증 목표는:

- A1/A2/A3 복수 robot
- 양방향 task 동시 요청
- Arbiter HOLD/GRANT
- 실제 RMF task submit
- VDA5050 이동
- Block/HB release
- opposite-direction overlap = 0
- 전체 mission completion

까지 E2E로 확인하는 것이다.
