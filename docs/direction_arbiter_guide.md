# Direction Arbiter 운영 및 검증 가이드

## 목적과 적용 위치

이 기능은 Open-RMF의 Planner나 Negotiation을 교체하지 않는다. 좁은 1차선 양방향 통로에서 반대 방향 Task가 동시에 RMF 계획 입력으로 들어가지 않도록, 기존 RMF API 앞에서 진입을 결정하는 Phase-A admission gate다.

```text
T4 Task → Task Gate(:8200) → 허가된 구간만 RMF API(:8100)
                                  ↓
VDA5050 State(MQTT) → RobotTracker → DirectionArbiter
```

Production 단계에서는 동일한 core state machine을 Fleet Adapter의 itinerary 확정 전 지점으로 옮겨야 한다.

## 안전 원칙

- Block과 목적 Holding Bay를 동시에 예약한다.
- 같은 Block에 반대 방향 점유·예약을 허용하지 않는다.
- 대기는 HB/Parking/Refuge에서만 수행한다.
- `agvPosition`을 우선 사용하고 `edgeStates`, `nodeStates`, `lastNodeId`는 보조 증거로 사용한다.
- Block 내부 telemetry가 끊기면 자동 EXIT하지 않고 `BLOCKED`로 유지한다.
- 방향은 route 설정에서만 가져오며 node 번호나 x좌표로 추측하지 않는다.
- `robot.py`, `order_manager.py`, RMF Core는 수정하지 않는다.

## 설정

`config/corridor_blocks.yaml`은 현재 checkpoint용 안전 기본값이며 `enabled: false`다. 실제 map에서 HB의 물리적 안전성과 좌표를 확인하기 전에는 켜지 않는다.

`config/corridor_blocks_segmented_example.yaml`은 세 Block 예제다. 다음 항목을 실제 navigation graph와 일치시켜야 한다.

- HB `node_id`, 원/영역 좌표, 실제 capacity
- Block 경계와 양방향 edge ID
- Task의 start/goal node와 Block 통과 순서
- 중간 HB는 본선 로봇 footprint와 겹치지 않는 off-lane 지점

각 Block의 `direction_domain` 기본값은 Block ID다. 반드시 함께 같은 방향으로 잠가야 하는 물리적 구간만 동일한 `direction_domain`을 명시한다.

## 실행

기존 T1(RMF Core), T2(Fleet Adapter), T3(Robot Simulator), MQTT broker를 먼저 실행한다. Python 환경에는 simulator `requirements.txt`를 설치한다.

```bash
cd rmf_simulation_workspace
python3 -m pip install -r rmf_dev_tool-main/vda5050_robot_simulator/requirements.txt
```

안전 OFF 회귀 확인:

```bash
./scripts/run_direction_arbiter.sh config/corridor_blocks.yaml
```

편집한 segmented 설정으로 실행:

```bash
./scripts/run_direction_arbiter.sh config/corridor_blocks_segmented_example.yaml
```

Task는 8100이 아니라 8200으로 보낸다.

```bash
./scripts/t4_dispatch_via_arbiter.sh AGV_A1 N3
```

상태 확인:

```bash
curl --noproxy '*' http://127.0.0.1:8200/traffic/status
```

강제 reset은 실제 Block 내부에 로봇이 없음을 현장에서 확인한 경우에만 사용한다.

```bash
curl --noproxy '*' -X POST 'http://127.0.0.1:8200/traffic/reset?force=true'
```

## 로그

`[TRAFFIC]` 접두어로 요청, 대기, 허가, Block 진입/이탈, 방향 전환, fault, Task hold/release를 확인한다. `TASK_HELD`는 아직 RMF에 제출되지 않은 상태다.

## 테스트

```bash
cd rmf_simulation_workspace
python3 -m unittest discover -s tests -v
python3 -m compileall traffic_control tests
bash -n scripts/run_direction_arbiter.sh scripts/t4_dispatch_via_arbiter.sh
```

자동 테스트에는 Manager OFF, 원자 예약, 반대 방향 WAIT, starvation batch closure, geometry/edge 점유, telemetry timeout, 동적 투입, 같은 방향 pipeline, 2v2 완료가 포함된다.

실제 통합 검증은 다음 순서로 한다.

1. 1v1
2. 2v1
3. 2v2
4. 3v3
5. 진행 중 반대 방향 신규 투입
6. 중간 HB full
7. Block 내부 telemetry 차단
8. 서로 떨어진 독립 Block의 반대 방향 동시 운용

각 실행에서 `endpoint_exchange_without_buffer`, `NO_PHYSICAL_ESCAPE`, negotiation/replan 횟수, timeout, 완료 시간과 대기 시간을 전후 비교한다.

## 알려진 제한

- 예제 좌표와 ID는 실제 P4 map 값이 아니다.
- Task Gate는 Phase-A PoC이며 RMF가 Task를 받은 뒤의 재계획까지 통제하지 않는다.
- Junction 충돌 자원과 Holding staging 이동은 아직 포함하지 않는다.
- upstream에 제출된 ACTIVE Task 취소는 구현하지 않는다.
- 실제 2v2/3v3 RMF 통합 결과는 RMF Core/API/Fleet Adapter/MQTT가 모두 실행되는 환경에서 별도 측정해야 한다.
