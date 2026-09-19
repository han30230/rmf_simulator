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
않는다.

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

1v3에서는 A1이 `CHAIN_SIDE_2`에 실제 도착한 뒤 B1/B2/B3가 각 왼쪽
종점으로 직행하고, 세 대가 통과한 뒤 A1이 오른쪽 종점으로 이동한다.
2v2에서는 A1/A2가 각각 SIDE2/SIDE1에 실제 도착한 뒤 B1/B2가 왼쪽으로
직행하며, 이후 A1/A2가 오른쪽 종점으로 이동한다. 반대 방향 로봇이 없는
clear chain에서는 최종 종점까지 하나의 작업으로 직행한다.

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
