# RMF + VDA5050 Passing Bay 실행 가이드

이 문서는 현재 `feature/p4-single-passing-bay-poc` 기준이다. 모든 경로는 저장소 위치를 자동으로 계산하므로 특정 PC의 절대경로에 의존하지 않는다.

## 자동 실행

최초 1회:

```bash
cd ~/rmf-work/rmf_passing_bay_poc
./scripts/setup_workspace.sh
```

Passing-bay 전체 스택 시작:

```bash
./scripts/start_p4_passing_bay.sh
```

작업 투입:

```bash
./scripts/t4_dispatch_passing_bay.sh
```

2대 대 1대 실행:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_2v1.sh
./scripts/t4_dispatch_passing_bay_2v1.sh
```

2대 대 1대에서는 A1이 2106을 통과하면 B1이 `6137 → 2101`로 출발한다.
B2는 A1의 현재 단계와 이후 단계를 포함한 반대 방향 작업이 끝날 때까지
HB_RIGHT에서 기다린다. A1이 2108에 도착하면 대기 중인 경로를 다시
평가하고, side bay를 거치지 않는 `2108 → 2101` 직행 작업을 제출한다.

2대 대 2대 실행:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_2v2.sh
./scripts/t4_dispatch_passing_bay_2v2.sh
```

A1/A2는 왼쪽에서 오른쪽으로, B1/B2는 오른쪽에서 왼쪽으로 이동한다.
선행 block이 비면 같은 방향의 다음 로봇이 먼저 이동하고, 공유 conflict
domain은 반대 방향 waiter가 생긴 시점에 현재 batch를 닫는다. B1은
`6137`에 먼저 들어간 뒤 A1/A2의 공유 충돌 구간 통과를 기다린다. 설정된
`requires_opposite_routes_cleared` 정책은 같은 direction domain에 남아 있는
반대 방향 현재·후속 route step을 확인하며, 실행 중인 step은 telemetry로
`release_node`를 통과한 뒤에만 안전하게 끝난 것으로 간주한다.

## 물리 staging slot 실행

동일한 2101/2108 좌표에 여러 로봇을 겹쳐 놓지 않는 2v2는 다음과 같이
실행한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_staging_2v2.sh
./scripts/launch_p4_passing_bay_staging_visualizer.sh
./scripts/t4_dispatch_passing_bay_staging_2v2.sh
```

1v3은 A1 한 대가 왼쪽에서 오른쪽으로, B1/B2/B3 세 대가 오른쪽에서
왼쪽으로 이동한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_staging_1v3.sh
./scripts/launch_p4_passing_bay_staging_visualizer.sh
./scripts/t4_dispatch_passing_bay_staging_1v3.sh
```

각 staging Holding Bay는 `capacity 1`이고 서로 다른 graph node와 좌표를
가진다. B1은 `P4_RS1 → 6137 → P4_LS1`로 이동한다. B2/B3는 A1의 반대
방향 작업이 끝날 때까지 각자의 오른쪽 slot에 머물고, 이후
`P4_LS2/P4_LS3`로 같은 방향 pipeline 주행을 한다. A1은 B1이 비운
`P4_RS1`을 목적지로 사용한다.

주행 도중 B2/B3 작업을 추가하려면 같은 staging 1v3 stack에서 다음을
실행한다.

```bash
./scripts/t4_dispatch_passing_bay_staging_dynamic_1v3.sh
```

A1/B1을 먼저 투입한 뒤 B1의 `HB_MIDDLE_SIDE` 도착 조건으로 B2를,
A1의 `last_node_id=2105` 조건으로 B3를 투입한다. timeout은 오류 감지용이며
작업 투입 시점은 고정 sleep이 아니라 `/traffic/status` 상태로 결정한다.

이 동작은 로봇 이름을 검사하는 production 분기가 아니라 YAML에 정의된
Holding Bay, block endpoint, direction domain과 route step으로 결정된다.
다른 현장에서는 slot 수와 route 조합을 설정으로 바꾸고, simulation 좌표는
차체 크기·제동거리·정지 오차를 반영해 측량한 좌표로 교체해야 한다.

## 다중 Corridor 실행

P4와 P5 두 개의 독립 통로를 동시에 실행한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_p5_multi_corridor.sh
./scripts/launch_p4_p5_multi_corridor_visualizer.sh
./scripts/t4_dispatch_p4_p5_multi_corridor.sh
```

Visualizer에는 서로 연결되지 않은 두 navigation graph가 표시된다. P4는
왼쪽에서 오른쪽으로 교행을 시작하고 P5는 오른쪽에서 왼쪽으로 직행을
시작하므로, 서로 반대인 direction domain이 동시에 활성화되는 것을 확인할
수 있다. 각 corridor는 고유한 holding bay, block ID, direction domain을
가지며 Arbiter의 판정 로직은 공유한다. 새 corridor나 통로당 최대 4대의
slot을 추가할 때는 map과 YAML route/capacity를 확장하고 production Python에
로봇 ID나 node ID 조건을 추가하지 않는다.

다중 corridor map:

```text
rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_p5_multi_passing_bay.yaml
```

핵심 로그 확인:

```bash
tail -F .runtime/arbiter.log |
grep --line-buffered -E \
  'TASK_RELEASED|ROBOT_CLEARED_BLOCK|TASK_COMPLETED|BLOCK_FAULT'
```

종료:

```bash
./scripts/stop_p4_passing_bay.sh
```

Docker container까지 모두 중지하려면 `--all`을 붙인다.

## 연결된 장거리 Corridor 실행

`connected_corridor_chain.yaml`은 서로 분리된 통로가 아니라 하나의 긴
양방향 1차선 본선이다. `C1 - SIDE1 - C2 - SIDE2 - C3` 구조이며 SIDE1과
SIDE2는 본선 junction 옆의 물리 사이드 베이다. 종점과 사이드 베이만
SafeStop으로 사용하므로 정상 스케줄링에서는 본선 위에 대기 작업을 만들지
않는다. 좌우의 L1-L4/R1-R4는 본선 아래쪽의 독립 junction에 연결된 leaf
slot이다. 한 slot으로 가는 경로가 다른 slot을 지나지 않으므로 대기·도착
로봇과 본선 주행 로봇의 물리 경로가 겹치지 않는다.

1대 대 3대:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_connected_corridor_chain.sh 1v3
./scripts/launch_connected_corridor_chain_visualizer.sh
./scripts/dispatch_connected_corridor_chain_1v3.sh
```

2대 대 2대:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_connected_corridor_chain.sh 2v2
./scripts/launch_connected_corridor_chain_visualizer.sh
./scripts/dispatch_connected_corridor_chain_2v2.sh
```

상태 기반 동적 2대 대 2대:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_connected_corridor_chain.sh 2v2
./scripts/launch_connected_corridor_chain_visualizer.sh
./scripts/dispatch_connected_corridor_chain_dynamic_2v2.sh
```

동적 시나리오는 A1만 먼저 제출한다. A1이 출발 slot을 벗어나면 B1을 추가하고,
B1이 ACTIVE가 되면 A2를, A2가 ACTIVE가 되면 B2를 추가한다. 반대 방향의 오래된
대기 요청이 현재 direction batch를 닫으므로 실제 ACTIVE 순서는
`A1 → B1 → A2 → B2`가 된다. 조건과 목적지는
`config/dynamic_connected_corridor_chain_2v2.yaml`에서 바꿀 수 있다.

현재 dispatch 예제는 먼저 허가된 같은 방향 batch가 최종 leaf slot까지
도착한 뒤 반대 방향 batch를 출발시킨다. 1v3은 A1이 R4에 도착한 뒤
B1/B2/B3가 L4/L3/L2로 이동하고, 2v2는 A1/A2가 R4/R3에 도착한 뒤
B1/B2가 L4/L3로 이동한다. 목적지는 dispatch 데이터이며 production Python은
이 robot ID나 node ID를 검사하지 않는다. 반대 방향 로봇이 없는 clear
chain에서는 사이드 베이에 들르지 않고 최종 leaf slot까지 직행한다.

상태와 이벤트는 다음 명령으로 확인한다.

```bash
curl -s --noproxy '*' http://127.0.0.1:8200/traffic/status |
python3 -m json.tool

tail -F .runtime/arbiter.log |
grep --line-buffered -E \
  'AUTHORITY_ADMIT|TASK_RELEASED|ROBOT_CLEARED_BLOCK|TASK_COMPLETED|BLOCK_FAULT|ROBOT_FAULT'
```

movement authority의 중간 블록은 telemetry에 따라 롤링 해제되지만 마지막
블록은 목적지 SafeStop 도착까지 유지된다. 이 규칙은 robot ID나 map node를
검사하는 분기가 아니라 authority의 블록 순서와 YAML topology에 적용된다.

`WAITING`/`RETRY` job은 `/traffic/jobs/{job_id}/cancel`로 취소한 뒤 새 목적지를
제출할 수 있다. 이미 RMF에 전달된 `ACTIVE` job은 이 endpoint가 HTTP 409를
반환한다. 운행 중 목적지 변경은 본선에서 즉시 반전시키지 않고 다음 SafeStop에
도착한 뒤 새 intent를 제출하는 정책으로 구현해야 한다.

## 구성요소와 포트

| 구성요소 | 역할 | 포트/통신 |
| --- | --- | --- |
| Mosquitto | VDA5050 MQTT broker | TCP 1883 |
| Robot Simulator | AGV_A1/B1/B2 state 발행, order 수행 | MQTT |
| Fleet Adapter | MQTT state/order와 RMF 변환 | ROS 2 + MQTT |
| RMF Schedule/Dispatcher | 경로 schedule과 task 배정 | ROS 2 |
| RMF API Server | REST task endpoint | HTTP 8100 |
| Direction Arbiter | Corridor/Holding Bay 진입 제어 | HTTP 8200 + MQTT |

## Passing-bay 상태 흐름

1. A1이 `2101 → 2104`로 이동한다.
2. B1이 `2108 → 6137`로 이동해 side bay에서 대기한다.
3. A1은 하나의 task로 `2104 → 2108`을 계속 주행한다.
4. A1 telemetry가 `lastNodeId=2106`을 보고하면 공유 conflict domain이 해제된다.
5. 1대 대 1대와 2대 대 1대에서는 A1이 2106에서 정지하지 않고 주행하며,
   B1은 `6137 → 2101`로 출발한다.
6. 2대 대 1대에서는 A1 완료 후 B2가 `2108 → 2101`로 직행한다.
7. 2대 대 2대에서는 B1이 6137에 머무는 동안 A2도 통과한다. A2가 설정된
   release node 2106을 지난 뒤 B1이 2101로 출발하고, 반대 방향 작업이 모두
   끝나면 B2가 side bay를 거치지 않고 직행한다.
8. 목적지 holding-bay 예약은 각 로봇이 실제 도착할 때 해제된다.

release-node telemetry가 누락되면 통로는 fail-closed 상태를 유지한다.

## GUI

```bash
cd ~/rmf-work/rmf_passing_bay_poc
source .venv/bin/activate
python rmf_dev_tool-main/vda5050_gui/vda5050_gui.py
```

외부 스크립트가 Simulator를 실행하므로 GUI에서는 **Monitor** 탭을 사용한다. 맵은 다음 파일을 연다.

```text
rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_passing_bay.yaml
```

## 수동 진단

```bash
docker compose -f rmf_platform-main/docker-compose.yml ps
docker logs --tail 200 vda5050_fleet_adapter
curl -s --noproxy '*' http://127.0.0.1:8200/traffic/status |
python3 -m json.tool
```

실행 파일과 자세한 새 PC 설치법은 저장소 루트의 `README.md`를 우선 기준으로 한다.

## Production과 시뮬레이션의 구분

기존 `start_p4_*`, `start_connected_corridor_chain.sh`은 로컬 Simulator와 개발용
MQTT/RMF 구성을 시작하는 시뮬레이션 명령이다. 실제 로봇 연결에는 이 launcher를
사용하지 않는다.

실차 profile은 다음 명령으로 먼저 검사한다.

```bash
.venv/bin/python scripts/validate_production_deployment.py /path/to/site-production.yaml
```

검사가 통과한 profile만 production launcher에 전달한다.

```bash
export RMF_API_BEARER_TOKEN='<site service token>'
./scripts/start_production_corridor.sh /path/to/site-production.yaml
```

production launcher는 Simulator와 Visualizer를 실행하지 않고 외부 MQTT broker,
RMF 서비스와 실제 Fleet Adapter만 사용한다. 모든 필수 로봇이 fresh telemetry로
configured SafeStop에 정지한 clean-start 상태가 확인되기 전에는 `/ready`가 503을
반환하고 Task Gate가 새 작업을 거부한다.

# VDA5050 장애 주입 검증

연결형 Corridor 시뮬레이터는 실제 로봇이 보내는 것과 같은 State 및
Connection 메시지에 장애를 주입할 수 있다. 기본 예제는 안전을 위해
`enabled: false`이며, 로봇과 노드 선택은 Python 코드가 아니라
`rmf_dev_tool-main/vda5050_robot_simulator/connected_corridor_fault_scenarios.yaml`
의 규칙으로 지정한다.

```bash
.venv/bin/python rmf_dev_tool-main/vda5050_robot_simulator/run.py \
  --config connected_corridor_chain_scenario.yaml \
  --fault-scenarios connected_corridor_fault_scenarios.yaml
```

지원하는 trigger는 `elapsed_at_least`, `at_node`, `driving`이며 함께 쓰면
모두 만족해야 발화한다. action은 E-stop, 운전 모드, pause, map ID,
positionInitialized, State 발행 중단, Connection 발행을 지원한다. 규칙은
한 번 발화하며 상태 효과는 뒤의 복구 규칙이 덮어쓸 때까지 유지된다.

실행 결과는 고정 대기 시간이 아니라 status 조건으로 판정한다.

```bash
.venv/bin/python scripts/run_connected_corridor_fault_scenario.py \
  --scenario inside_estop --robot AGV_A1 \
  --log .runtime/arbiter.log --timeout 120
```

Corridor 내부의 E-stop 또는 State timeout은 로봇과 관련 Block을 fault로
잠그고 반대편 authority를 허용하지 않아야 한다. SafeStop의 MANUAL은 해당
로봇의 새 authority만 막아야 하며 Block fault를 만들면 안 된다. 재시작 때
Corridor 내부 로봇을 관측했다면 로봇이 나중에 SafeStop으로 보이더라도
운영자 복구 전에는 `recovery.required`가 유지되어야 한다.

## 2026-09-21 실제 시뮬레이션 결과

- 정상 동적 2v2는 `AGV_A1 → AGV_B1 → AGV_A2 → AGV_B2` 순서로
  활성화됐고, 최종 위치는 각각 `CHAIN_R4`, `CHAIN_L4`, `CHAIN_R3`,
  `CHAIN_L3`였다. 네 job이 모두 COMPLETE였고 C1/C2/C3는 FREE,
  최소 관측 로봇 간 거리는 7.2m였다.
- C2 내부 E-stop은 A1을 `safety.estop`으로 ineligible 처리하고 미해제
  C2/C3를 fault로 잠갔다. 반대 로봇 authority는 발급되지 않았다.
- C2 내부 State 발행 중단은 `state.stale` 이후 `telemetry_timeout`으로
  A1과 C2/C3를 잠갔다. Connection 발행은 State 억제와 독립적으로 유지됐다.
- clean-start 이후 A1이 SafeStop에서 MANUAL로 바뀌면 A1 요청만
  `robot_not_eligible`로 거절됐고 A2 요청은 ADMIT됐다. Block fault는 없었다.
- Corridor 내부 로봇이 있는 상태에서 Arbiter를 재시작하면
  `recovery.required`가 고정됐으며, 로봇이 종점 SafeStop에 도착해도 자동으로
  task admission을 재개하지 않았다.

VDA5050 Connection은 주기 heartbeat가 아니라 retained 상태 이벤트로 취급한다.
따라서 오래된 `ONLINE` 수신 시각만으로 offline 판정을 내리지 않으며,
명시적 `OFFLINE`/`CONNECTIONBROKEN` 또는 State timeout을 안전 정지 근거로 쓴다.
