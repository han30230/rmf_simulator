# P4 단일 Passing Bay PoC 설계

## 1. 목적

현재 `2101 <-> 2108` 전체를 하나의 양방향 블록으로 제어하면 반대 방향 로봇은 출발점에서 기다린다. 이번 PoC는 통로 중앙에 메인 차선 밖 side bay를 하나 추가하여 다음 동작을 검증한다.

1. 양쪽 로봇이 충돌하지 않는 접근 구간까지 동시에 전진한다.
2. 한 로봇이 중앙 side bay로 완전히 빠진다.
3. 반대 방향 로봇이 메인 차선으로 통과한다.
4. 통과 완료 후 side bay의 로봇이 다시 합류하여 목적지로 이동한다.

이 설계는 홈 시뮬레이션용 2대 PoC다. 실제 로봇용 일반 해법이나 여러 passing bay 중 최적 지점을 선택하는 기능은 범위에 포함하지 않는다.

## 2. 성공 시나리오

- A1: `2101`에서 `2108`로 이동한다.
- B1: `2108`에서 `2101`로 이동한다.
- 두 요청은 1초 이내 간격으로 A1, B1 순서로 제출한다.
- A1은 서쪽 접근 대기점 `2104`까지 먼저 전진한다.
- B1은 중앙 분기 `2105`를 거쳐 side bay `6137`로 들어간다.
- B1이 `6137`에 정지한 것이 위치와 `lastNodeId`로 확인될 때까지 A1은 `2104`에서 기다린다.
- B1이 side bay에 들어가면 A1은 `2104 -> 2108` 구간을 통과한다.
- A1이 `2108`에 도착하고 공유 통행 도메인이 비면 B1은 `6137 -> 2101`로 재출발한다.

예상된 upstream 단계 목표 순서는 다음과 같다.

```text
A1 -> 2104
B1 -> 6137
A1 -> 2108
B1 -> 2101
```

## 3. 맵 토폴로지

기존 메인 노드와 좌표를 유지하고 `6137`만 side bay로 추가한다.

| 역할 | 노드 | 좌표 | 비고 |
|---|---|---|---|
| 좌측 종점 | `2101` | `(6.833, 92.871)` | `HB_LEFT` |
| 서측 접근 대기점 | `2104` | `(30.424, 92.871)` | `HB_WEST_GATE` |
| 중앙 분기 | `2105` | `(42.508, 92.871)` | 메인 차선, 대기 금지 |
| 중앙 side bay | `6137` | `(42.508, 91.100)` | `HB_MIDDLE_SIDE`, 용량 1 |
| 우측 종점 | `2108` | `(73.7274, 92.8248)` | `HB_RIGHT` |

추가 lane은 `2105 -> 6137`과 `6137 -> 2105` 두 개다. `6137`의 원형 geometry는 메인 코리도 geometry와 겹치지 않게 잡아, 로봇이 실제로 차선 밖으로 빠졌을 때만 holding bay 도착으로 판정한다.

`2104`는 중앙 분기에서 약 12 m 떨어져 있어 B1이 `2105 -> 6137`로 진입하는 동안 A1이 메인 합류부를 침범하지 않는 접근 정지점으로 사용한다.

## 4. 블록과 방향 도메인

### 4.1 독립 접근 블록

`P4_WEST_ADVANCE`는 `HB_LEFT -> HB_WEST_GATE` 구간이다. A1이 반대편 로봇과 무관하게 중앙 직전까지 전진하도록 허용한다.

### 4.2 공유 Passing Event 도메인

다음 세 논리 블록은 모두 동일한 `direction_domain: P4_PASSING_EVENT`를 사용한다.

| 블록 | 동작 | 방향 |
|---|---|---|
| `P4_EAST_TO_SIDE` | `HB_RIGHT -> HB_MIDDLE_SIDE` | `B_TO_A` |
| `P4_GATE_TO_RIGHT` | `HB_WEST_GATE -> HB_RIGHT` | `A_TO_B` |
| `P4_SIDE_TO_LEFT` | `HB_MIDDLE_SIDE -> HB_LEFT` | `B_TO_A` |

동일 도메인의 기존 batch-close 규칙으로 다음 순서를 만든다.

1. B1이 `P4_EAST_TO_SIDE`를 B_TO_A 방향으로 사용한다.
2. A1이 `P4_GATE_TO_RIGHT`를 요청하면 반대 방향 waiter가 생겨 현재 B_TO_A batch가 닫힌다.
3. B1이 side bay에 들어가 첫 블록을 비워도 B1의 다음 B_TO_A 단계는 연속 승인되지 않는다.
4. 비워진 도메인이 A_TO_B로 전환되어 A1을 통과시킨다.
5. A1이 통과한 뒤 도메인이 다시 B_TO_A로 전환되어 B1을 재출발시킨다.

이 PoC의 핵심은 B1이 side bay에 도착하기 전에 A1이 `HB_WEST_GATE`에 도착하여 A_TO_B waiter로 등록되는 것이다. 이를 위해 A1을 먼저 제출하고 B1을 1초 이내에 제출하며, `2101 -> 2104` 거리를 `2108 -> 6137`보다 짧게 유지한다.

## 5. Route 구성

### `P4_LEFT_TO_RIGHT_VIA_GATE`

1. `HB_LEFT -> HB_WEST_GATE`, 목표 `2104`
2. `HB_WEST_GATE -> HB_RIGHT`, 목표 `2108`

### `P4_RIGHT_TO_LEFT_VIA_SIDE`

1. `HB_RIGHT -> HB_MIDDLE_SIDE`, 목표 `6137`
2. `HB_MIDDLE_SIDE -> HB_LEFT`, 목표 `2101`

첫 PoC에서는 오른쪽에서 왼쪽으로 가는 로봇이 항상 side bay를 경유한다. 반대 로봇이 없을 때 bay를 생략하는 동적 경로 선택은 후속 기능이다.

## 6. 안전 규칙

- 모든 중간 holding bay 용량은 1이다.
- `unmatched_route_policy`는 `BLOCKED`를 유지한다.
- 로봇은 좌표가 holding geometry 안에 있고 `driving=false`일 때만 다음 단계로 진행한다.
- telemetry가 코리도 내부에서 timeout되면 해당 블록은 fail-closed로 유지한다.
- 예약 없는 로봇이 블록 geometry 안에서 검출되면 기존 `unexpected occupancy` fault를 발생시킨다.
- A1이 B1보다 먼저 공유 도메인을 선점하면 B1은 `2108`에서 대기한다. 이는 효율은 낮아도 안전한 fallback이다.
- 이 구성은 2대 PoC 전용이다. 세 번째 로봇을 투입하면 논리 블록의 중첩 geometry와 접근 블록 간 충돌 자원을 별도로 일반화해야 한다.

## 7. 구현 파일

기존 정상 동작 파일을 덮어쓰지 않고 passing-bay 전용 파일을 추가한다.

- `rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_passing_bay.yaml`
- `rmf_platform-main/docker-compose.p4-passing-bay.yml`
- `config/corridor_blocks_p4_passing_bay.yaml`
- `tests/test_passing_bay_config.py`
- `tests/test_passing_bay_flow.py`
- `scripts/t4_dispatch_passing_bay.sh`

시뮬레이터의 기존 `p4_scenario.yaml`은 A1/B1 시작 좌표가 그대로이므로 재사용한다. B2는 첫 PoC 동안 작업을 제출하지 않는다.

## 8. 테스트

### 정적 설정 테스트

- `6137`이 메인 노드 `2105`와 양방향으로 연결된다.
- `HB_MIDDLE_SIDE` geometry가 메인 코리도 geometry와 겹치지 않는다.
- 두 관리 route가 각각 두 단계이고 모든 step의 source/destination이 block endpoint와 일치한다.
- 세 passing block이 동일한 `P4_PASSING_EVENT` direction domain을 사용한다.

### 상태 머신 테스트

1. A1을 `2101`에, B1을 `2108`에 배치한다.
2. A1 요청 결과가 `ADMIT`, 첫 upstream 목표가 `2104`인지 확인한다.
3. B1 요청 결과가 `ADMIT`, 첫 upstream 목표가 `6137`인지 확인한다.
4. A1이 `2104`에 도착하면 두 번째 단계가 `WAIT`인지 확인한다.
5. B1이 `6137`에 도착하면 B1의 두 번째 단계는 계속 `WAIT`, A1 목표 `2108`만 전달되는지 확인한다.
6. A1이 `2108`에 도착하면 B1 목표 `2101`이 전달되는지 확인한다.
7. 두 작업이 `COMPLETE`, 모든 블록이 `FREE`, 모든 reservation/waiting queue가 비었는지 확인한다.

### 런타임 성공 기준

- 반대 방향 로봇의 같은 메인 구간 동시 점유 0건
- B1이 `6137`에서 `driving=false`, `current_hb=HB_MIDDLE_SIDE` 상태로 실제 대기
- A1이 B1 대기 중 중앙 메인 차선을 통과
- A1 도착 후 B1 자동 재출발
- 최종 작업 2개 모두 `COMPLETE`
- fault와 stale reservation 0건

## 9. 알려진 제한사항과 후속 단계

- 첫 PoC는 비대칭이다. 오른쪽 출발 로봇만 side bay를 사용한다.
- 동적 충돌이 없더라도 B_TO_A route는 side bay를 경유한다.
- 여러 side bay 중 최근접 베이 선택, ETA 기반 우선순위, 3대 이상 pipeline은 포함하지 않는다.
- VDA5050 시뮬레이터가 첫 edge orientation 없이 한 노드를 이동한 뒤 회전하는 알려진 시각화 문제가 있으며, traffic admission 결과와 분리하여 후속 수정한다.
- PoC 성공 후 route alternative와 shared conflict resource를 도입하여 방향 대칭 및 다중 베이 선택으로 일반화한다.
