# P4 무정지 조기 해제 설계

## 1. 목적

현재 single passing-bay PoC에서 A1은 `2104 -> 2108` 작업 전체가 끝날 때까지 `P4_PASSING_EVENT` 방향 도메인을 점유한다. 그래서 B1은 side bay `6137`에 안전하게 빠져 있어도 A1이 `2108`에 도착할 때까지 출발하지 못한다.

이번 변경은 A1의 RMF 작업을 나누지 않고 `2104 -> 2108`로 계속 실행하되, A1이 충돌 구간의 동쪽 안전 경계인 `2106`을 통과했다는 VDA5050 telemetry가 확인되면 공유 방향 도메인만 조기에 해제한다. 그 즉시 B1은 `6137 -> 2101` 작업을 시작할 수 있다. A1은 `2106`에서 정지하거나 새 작업을 받지 않는다.

## 2. 성공 동작

1. A1은 `2101 -> 2104`, B1은 `2108 -> 6137`로 이동한다.
2. B1이 side bay `6137`에 정지하면 A1의 단일 upstream 작업 `2104 -> 2108`이 시작된다.
3. A1이 `2105`에 있을 때 B1은 계속 기다린다.
4. A1이 `2106`을 통과하고 `lastNodeId=2106`이 관측되면 A1의 공유 충돌 블록만 `CLEARED`가 된다.
5. A1은 `driving=true` 상태로 `2108`까지 계속 이동하며, B1은 동시에 `6137 -> 2101`로 출발한다.
6. A1의 `HB_RIGHT` 도착 예약은 `2108`에 실제 도착할 때까지 유지된다.
7. 두 작업이 완료되면 모든 block, grant, holding-bay reservation, waiting queue가 비어 있어야 한다.

예상 upstream 목표 순서는 기존과 같다.

```text
A1 -> 2104
B1 -> 6137
A1 -> 2108
B1 -> 2101
```

차이는 네 번째 작업이 A1의 `2108` 도착 후가 아니라 A1의 `2106` 통과 후 전달된다는 점이다.

## 3. 구성 인터페이스

`RouteStep`에 선택 항목 `release_node`를 추가한다.

```yaml
- block_id: P4_GATE_TO_RIGHT
  direction: A_TO_B
  source_hb: HB_WEST_GATE
  destination_hb: HB_RIGHT
  goal_node: "2108"
  release_node: "2106"
```

의미는 다음과 같다.

- `goal_node`는 upstream RMF 작업 목적지이며 변경되지 않는다.
- `release_node`는 해당 step이 가진 block 및 direction-domain 충돌 자원을 조기에 반환할 수 있는 경계다.
- destination holding bay 예약은 조기 반환 대상이 아니다.
- `release_node`가 없는 기존 route는 현재처럼 `goal_node` 도착까지 자원을 유지한다.

Registry는 `release_node`가 빈 문자열이 아니고, `goal_node`와 다르며, 해당 방향의 block edge 목록에 포함된 노드인지 검증한다. 이 검증은 오타로 충돌 경계를 임의의 위치에서 해제하는 것을 막는다.

P4 설정에서는 `P4_GATE_TO_RIGHT` A_TO_B step에만 `release_node: "2106"`을 둔다. production Python 코드에는 `AGV_A1`, `2106`, P4 block ID를 하드코딩하지 않는다.

## 4. 상태 모델과 자원 수명

`RobotCorridorState`에 `CLEARED`를 추가하여 다음 두 수명을 분리한다.

| 자원 | `2106` 통과 시 | `2108` 도착 시 |
|---|---|---|
| block occupant/reservation | 해제 | 이미 해제됨 |
| direction domain | 재조정하여 반대 방향 승인 가능 | 영향 없음 |
| destination HB 예약 | 유지 | 해제 후 occupant로 전환 |
| route step / upstream task | `ACTIVE` 유지 | `COMPLETE` 또는 다음 step 진행 |
| retained grant metadata | 유지 | 제거 |

`DirectionArbiter.mark_cleared(robot_id, block_id)`는 block의 occupant/reservation을 제거하고 robot state를 `CLEARED`로 바꾼다. grant metadata와 destination HB 예약은 보존한다. 이후 같은 direction domain을 즉시 reconcile하여 기다리던 반대 방향 요청을 승인할 수 있다.

`DirectionArbiter.mark_arrived(robot_id, block_id)`는 destination 도착 시 남아 있는 grant와 HB reservation을 정리하고 destination occupant를 확정한다. 일반 step은 기존 geometry/HB 기반 `mark_exited`에서 이미 정리되므로 이 호출은 idempotent해야 한다.

`active_block_for_robot()`은 `CLEARED` grant를 물리적 활성 block으로 반환하지 않는다. 반면 destination HB 조회는 retained grant를 계속 사용하여 A1이 `HB_RIGHT`에 들어가는 것을 정상 도착으로 판정한다.

## 5. Telemetry 처리

`RobotTracker`는 매 state message에서 다음 순서를 따른다.

1. 기존 position/edge 기반으로 block 진입 및 점유를 갱신한다.
2. 활성 grant에 `release_node`가 있고 `lastNodeId`가 정확히 일치하면 `mark_cleared`를 호출한다.
3. 조기 해제된 로봇의 `current_block`을 `None`으로 바꾸되, 중간 node를 holding bay로 간주하지 않는다.
4. 최종 destination holding bay 도착은 기존 `position + lastNodeId + driving=false` 조건으로 판단한다.

`lastNodeId`가 `2105`인 상태, 단순히 `2106` 방향으로 주행 중인 상태, position만 경계 밖으로 나온 상태로는 조기 해제하지 않는다. 설정된 node를 실제로 통과했다는 node telemetry가 필요하다.

공유 block geometry는 실제 충돌 구역인 `HB_WEST_GATE` 부근부터 `2106`까지로 축소한다. `2106` 동쪽의 A1은 조기 해제 후 managed geometry 밖에 있어 `unreserved_robot_detected_inside` 오탐이 발생하지 않아야 한다. B1의 `6137 -> 2101` grant는 중앙 충돌 구역을 포함하므로 A1과 동시에 같은 충돌 geometry를 점유할 수 없다.

## 6. TaskGate 처리

`TaskGate`가 step admission을 요청할 때 `release_node`를 Arbiter reservation으로 전달한다. A1이 `2106`을 통과해도 job 상태와 `step_index`는 변경하지 않는다. upstream에 추가 작업을 보내지 않으므로 A1은 멈추지 않는다.

`tracker.has_arrived(robot_id, goal_node)`가 참이 된 뒤 step을 증가시키기 전에 `arbiter.mark_arrived(robot_id, block_id)`를 호출한다. 이 동작은 조기 해제 step의 retained grant를 정리하며, 일반 step에는 안전한 no-op이다.

## 7. 안전 동작

- `release_node` telemetry가 오지 않으면 기존처럼 자원을 계속 유지한다. 시간이나 추정 위치만으로 해제하지 않는다.
- release 전에 telemetry가 끊기면 현재 block 점유가 유지되고 기존 timeout fault가 발생한다.
- release 후 destination까지 telemetry가 끊기면 공유 충돌 구역은 이미 물리적으로 지난 것으로 간주하지만 `HB_RIGHT` 예약은 계속 유지한다.
- 조기 해제 후 로봇이 비정상적으로 충돌 geometry 안으로 역진입하면 기존 unexpected-occupancy fault를 발생시킨다.
- destination HB 용량 검사는 최종 도착 때까지 유지되어 A1의 목적지를 다른 로봇이 선점하지 못한다.
- reset은 `CLEARED` grant와 HB reservation도 모두 제거한다.
- cancel 정책은 기존과 동일하다. upstream에 전달되어 `ACTIVE`인 step은 gate에서 취소하지 않는다.

이 기능은 교통 스케줄링 계층의 안전 여유를 높이는 기능이지, 실제 로봇의 비상 정지나 장애물 감지 기능을 대체하지 않는다. 현장 적용 전에는 실제 VDA5050 장비가 node 통과 직후 `lastNodeId`를 안정적으로 갱신하는지 확인해야 한다.

## 8. 관측 가능성

조기 해제 시 다음 로그를 한 번 기록한다.

```text
[TRAFFIC] ROBOT_CLEARED_BLOCK robot=<id> block=<id> release_node=<node>
```

`/traffic/status`의 block은 조기 해제 직후 A1 occupant/reservation을 포함하지 않아야 하고, `HB_RIGHT.reservations`에는 A1이 남아 있어야 한다. Robot snapshot의 `current_block`은 `None`, `last_node_id`는 `2106`, `driving`은 실제 telemetry 값인 `true`로 표시된다.

## 9. 테스트 전략

### Registry와 상태 머신

- `release_node`가 route step과 reservation에 전달되는지 확인한다.
- block edge에 없는 release node와 goal과 같은 release node를 거부한다.
- `mark_cleared`가 block/domain만 해제하고 destination HB 예약은 유지하는지 확인한다.
- `active_block_for_robot`이 `CLEARED` grant를 반환하지 않는지 확인한다.
- `mark_arrived`가 retained grant와 HB 예약을 정리하며 반복 호출에도 안전한지 확인한다.

### 1 대 1 통합 흐름

- A1이 `2105`에 있을 때 B1 job은 `WAITING`이고 B1의 두 번째 upstream 목표는 아직 없다.
- A1이 `lastNodeId=2106`, `driving=true`가 되면 B1의 `2101` 목표가 전달된다.
- 그 시점에도 A1 job은 `ACTIVE`, 목표는 여전히 `2108`, A1 telemetry는 `driving=true`다.
- A1이 `2107` 방향으로 이동하는 동안 B1이 중앙에서 서쪽으로 이동할 수 있다.
- 두 로봇이 목적지에 도착하면 두 job은 `COMPLETE`, 모든 교통 자원은 비어 있고 block fault가 없다.

### 회귀와 fail-closed

- `release_node`가 없는 기존 테스트는 기존 완료 시점까지 block을 유지한다.
- release node를 관측하지 않고 telemetry timeout이 나면 opposite waiter가 승인되지 않는다.
- 조기 해제 뒤 destination HB 예약이 다른 robot admission에 의해 침범되지 않는다.
- 전체 unittest suite를 실행하여 기존 single-corridor, fault, dynamic insert, passing-bay, orientation 동작을 모두 확인한다.

## 10. 범위와 후속 작업

이번 변경은 configurable early-clear primitive와 P4 1 대 1 runtime 검증까지만 포함한다. 다음은 별도 단계다.

- 2 대 1, 2 대 2 batch 및 HB capacity 정책
- 여러 corridor와 여러 passing bay 간 경로 선택
- 양쪽 방향에 대칭적인 release boundary 구성
- ETA 기반 우선순위와 starvation 제한
- 실제 로봇의 surveyed geometry, 정지 거리, localization 오차를 반영한 release node 선정

