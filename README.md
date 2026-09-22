# RMF + VDA5050 Direction Arbiter Simulation

Open-RMF, VDA5050 Fleet Adapter, MQTT Robot Simulator와 Direction Arbiter를 묶은 재현 가능한 시뮬레이션 작업공간이다. 현재 대표 시나리오는 P4 단일차선 중앙 Passing Bay에서 A1과 B1이 교행하는 PoC다.

## 현재 검증된 동작

- A1: `2101 → 2104 → 2108`
- B1: `2108 → 6137(side bay) → 2101`
- B1은 A1이 공유 구간을 지나는 동안 `6137`에서 대기
- A1은 `2106`에서 정지하지 않고 `2108`까지 하나의 RMF task로 계속 주행
- A1의 telemetry가 정확히 `lastNodeId=2106`을 보고하면 공유 conflict domain만 조기 해제
- 목적지 `HB_RIGHT` 예약은 A1이 실제 `2108`에 도착할 때까지 유지
- B1은 A1의 `2108` 도착 전, A1이 `2106`을 통과한 직후 출발 가능

단위·상태기계·통합 구성 테스트와 portable workspace 테스트를 포함한다.

## 요구 환경

- Windows 11 + WSL2 `Ubuntu-24.04` 또는 Ubuntu 24.04
- Docker Desktop의 WSL integration
- Python 3.12 권장, `python3-venv`, `curl`
- 인터넷 연결: 최초 Docker image와 Python package 다운로드에 필요

WSL 배포판 이름은 정확히 확인한다.

```powershell
wsl --list --verbose
wsl -d Ubuntu-24.04
```

## 새 PC Quick Start

```bash
mkdir -p ~/rmf-work
cd ~/rmf-work

gh repo clone han30230/rmf_simulator \
  rmf_passing_bay_poc \
  -- \
  --branch feature/p4-single-passing-bay-poc \
  --single-branch

cd ~/rmf-work/rmf_passing_bay_poc

./scripts/setup_workspace.sh
./scripts/start_p4_passing_bay.sh
./scripts/t4_dispatch_passing_bay.sh
```

2대 대 1대 시나리오는 별도 시작/투입 스크립트를 사용한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_2v1.sh
./scripts/t4_dispatch_passing_bay_2v1.sh
```

이 시나리오는 A1이 `2101 → 2108`, B1/B2가 `2108 → 2101`로 이동한다.
B1은 A1이 2106을 통과하면 side bay에서 출발하고, B2는 A1이 2108에
도착해 반대 방향 작업이 끝나면 side bay를 거치지 않고 `2108 → 2101`로
직행한다.

2대 대 2대 시나리오는 네 로봇을 등록하는 별도 스크립트를 사용한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_2v2.sh
./scripts/t4_dispatch_passing_bay_2v2.sh
```

A1/A2는 `2101 → 2108`, B1/B2는 `2108 → 2101`로 이동한다. 같은 방향
로봇은 안전한 선행 block까지 파이프라인으로 이동하고, 반대 방향 waiter가
생기면 현재 batch를 닫아 방향을 전환한다. B1은 먼저 `6137`에 진입하지만
A1뿐 아니라 같은 방향 후속 로봇 A2가 공유 충돌 구간의 `release_node`를
통과할 때까지 side bay에서 기다린다. 이후 B1이 `2101`로 출발하며, B2는
반대 방향 작업이 모두 끝난 뒤 `2108 → 2101`로 직행한다.

## 물리 staging slot 시나리오

기본 P4 시나리오는 회귀 검증을 위해 보존한다. 실제 로봇처럼 시작점과
목적지에서 서로 겹치지 않는 검증은 staging 전용 stack을 사용한다. 좌우에
각각 세 개의 개별 slot이 있고 각 Holding Bay의 `capacity 1`을 Arbiter가
관리한다.

staging 2v2 실행:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_staging_2v2.sh
./scripts/launch_p4_passing_bay_staging_visualizer.sh
./scripts/t4_dispatch_passing_bay_staging_2v2.sh
```

A1/A2는 `P4_LS1/P4_LS2`에서 출발해 `P4_RS1/P4_RS3`에 도착하고,
B1/B2는 `P4_RS1/P4_RS2`에서 출발해 `P4_LS1/P4_LS2`에 도착한다.
B1은 A1과 A2가 충돌 경계를 통과할 때까지 6137에 머문다.

staging 1v3 실행:

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_passing_bay_staging_1v3.sh
./scripts/launch_p4_passing_bay_staging_visualizer.sh
./scripts/t4_dispatch_passing_bay_staging_1v3.sh
```

A1은 왼쪽에서 오른쪽으로 이동한다. B1은 먼저 6137로 피한 뒤 A1의
`release_node` 통과 후 왼쪽으로 출발하고, B2/B3는 반대 방향 작업이
끝나면 서로 다른 목적지 slot로 직행한다.

주행 중에 새 작업이 들어오는 동적 1v3은 같은 stack을 시작한 뒤 아래
dispatch 스크립트를 사용한다.

```bash
./scripts/t4_dispatch_passing_bay_staging_dynamic_1v3.sh
```

이 스크립트는 A1/B1을 먼저 투입하고, status API를 polling하여 B1이
`HB_MIDDLE_SIDE`에 도착하면 B2를, A1이 2105에 도달하면 B3를 투입한다.
고정 sleep으로 순서를 만들지 않으므로 실제 주행 속도가 달라도 상태 전이를
기준으로 동작한다.

slot 수, 좌표, block, direction domain과 route는
`config/corridor_blocks_p4_passing_bay_staging.yaml` 및 staging map에 있다.
production Python은 로봇 이름이나 1v3 대수를 검사하지 않는다. 실제 현장에
적용할 때는 로봇 외형, 제동거리, 위치 오차와 안전 여유를 반영한 측량
좌표로 simulation 값을 교체해야 한다.

## 다중 Corridor 시나리오

서로 연결되지 않은 P4/P5 두 통로를 한 Arbiter에서 동시에 운용할 수 있다.

## 연결형 장거리 단일 통로

실제 팹처럼 하나의 긴 본선을 세 구간(C1/C2/C3)으로 나누고, 구간 사이에
사이드 베이 두 곳을 둔 연결형 예제도 제공한다. 로봇은 본선에서 대기하지
않으며, Arbiter가 현재 점유와 반대 방향 대기열을 보고 도달 가능한 가장 먼
SafeStop까지 여러 블록을 하나의 movement authority로 원자적으로 예약한다.
좌우 종점의 로봇 주차 위치는 본선 위에 연달아 놓지 않고, 각각 별도 junction에
연결된 본선 아래쪽 capacity-one leaf slot으로 구성한다. 따라서 대기 중인
로봇이나 이미 도착한 로봇을 다른 로봇의 경로가 관통하지 않는다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_connected_corridor_chain.sh 1v3  # 또는 2v2
./scripts/launch_connected_corridor_chain_visualizer.sh

# 별도 터미널
./scripts/dispatch_connected_corridor_chain_1v3.sh  # 또는 ..._2v2.sh
```

실행 중 반대 방향 작업을 추가하면서 A1→B1→A2→B2 순서로 교대시키려면
2v2 runtime에서 다음을 실행한다.

```bash
./scripts/dispatch_connected_corridor_chain_dynamic_2v2.sh
```

투입 순서와 조건은 `config/dynamic_connected_corridor_chain_2v2.yaml`에
있다. 범용 runner는 고정 sleep 대신 robot holding-bay 이탈과 job 상태를
polling하며, 각 조건은 독립 timeout을 사용한다.

중간 블록은 로봇이 다음 구간으로 넘어가면 순서대로 해제한다. 권한의 마지막
블록은 release node를 지나도 유지하며, 목적지 종점 또는 사이드 베이 안에
들어온 telemetry가 확인된 뒤에만 해제한다. 따라서 반대 방향 batch는 피신
로봇이 본선 junction에 도달한 시점이 아니라 실제 사이드 베이 도착 이후에
출발한다.

설정은 `config/corridor_blocks_connected_chain.yaml`, navigation graph는
`rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/connected_corridor_chain.yaml`
이다. 로봇 수, 로봇 이름, C1/C2/C3 node 이름은 production Python 정책에
포함되지 않으며, 현장 topology와 SafeStop/slot은 YAML로 정의한다.
Visualizer는 내부 node ID를 바꾸지 않고 같은 map에 공통인 접두사만 화면에서
줄여 표시한다. 예를 들어 `CHAIN_RJ1`은 `RJ1`로 보여 긴 label의 겹침을 줄인다.

아직 RMF에 전달되지 않은 `WAITING` 또는 `RETRY` job은 cancel한 뒤 새 목적지로
다시 제출할 수 있다. `ACTIVE` job은 본선에서 즉시 방향을 바꾸지 않도록 cancel을
거부한다. 현장용 reroute는 다음 configured SafeStop 도착 후 새 작업을 적용하는
방식으로 확장해야 한다.

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_p4_p5_multi_corridor.sh
./scripts/launch_p4_p5_multi_corridor_visualizer.sh
./scripts/t4_dispatch_p4_p5_multi_corridor.sh
```

P4와 P5는 서로 다른 holding bay, block 및 direction domain을 사용한다.
따라서 P4가 `A_TO_B`인 동안 P5가 `B_TO_A`로 주행할 수 있으며 한 통로의
대기열이 다른 통로의 방향 전환을 막지 않는다. 현재 실행 예제는 통로당
2대를 배치한다. 한 통로에서 최대 4대를 운용할 때도 production Python을
분기하지 않고 해당 통로의 slot, capacity, route를 YAML에 추가한다.

구성 파일은 `config/corridor_blocks_p4_p5_multi.yaml`, navigation graph는
`rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_p5_multi_passing_bay.yaml`이다.

`setup_workspace.sh`는 다음을 준비한다.

- 저장소 내부 `.venv`
- Arbiter, Simulator, GUI Python 의존성
- `rmf-core:latest`, `rmf-api-server:latest` Docker image alias
- 전체 Python 테스트

`start_p4_passing_bay.sh`는 다음을 자동 실행한다.

- MQTT가 없을 때만 Mosquitto 실행
- RMF Traffic Schedule, Dispatcher, Blockade, API Server 실행
- 기본 AGV_A1/AGV_B1 2대 Simulator 실행 또는 인자로 선택한 로봇 실행
- Passing-bay Fleet Adapter 실행
- 개발용 단기 JWT를 메모리에 생성
- `config/corridor_blocks_p4_passing_bay.yaml` Arbiter 실행

Docker image 자체는 수 GB이므로 Git에 저장하지 않는다. 기본 source image는 아래와 같으며 환경변수로 바꿀 수 있다.

- `ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest`
- `ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly`

## 실행 확인

상태:

```bash
curl -s --noproxy '*' \
  http://127.0.0.1:8200/traffic/status |
python3 -m json.tool
```

핵심 이벤트:

```bash
tail -F .runtime/arbiter.log |
grep --line-buffered -E \
  'TASK_RELEASED|ROBOT_CLEARED_BLOCK|TASK_COMPLETED|BLOCK_FAULT'
```

정상 순서는 대략 다음과 같다.

1. A1이 2104까지 이동하고 이어서 2108 task를 받음
2. B1이 6137 side bay로 이동 후 대기
3. A1이 2106을 통과하면서 `ROBOT_CLEARED_BLOCK`
4. A1이 계속 주행 중인 상태에서 B1의 2101 task가 `TASK_RELEASED`
5. A1과 B1이 각각 목적지에 도착하고 두 Job이 `COMPLETE`

종료:

```bash
./scripts/stop_p4_passing_bay.sh

# RMF/MQTT Docker container까지 모두 중지할 때
./scripts/stop_p4_passing_bay.sh --all
```

## GUI

WSLg가 활성화된 `Ubuntu-24.04`에서 실행한다.

```bash
cd ~/rmf-work/rmf_passing_bay_poc
source .venv/bin/activate
python rmf_dev_tool-main/vda5050_gui/vda5050_gui.py
```

외부 스크립트로 Simulator를 시작했다면 GUI의 **Monitor** 탭에서 A1/B1을 확인한다. **Simulation** 탭은 GUI 자체가 Simulator를 시작할 때 사용한다.

## 수동 실행 구성

자동 스크립트 대신 구성 요소를 따로 실행할 수도 있다.

- 기본 RMF Compose: `rmf_platform-main/docker-compose.yml`
- P4 baseline override: `rmf_platform-main/docker-compose.p4.yml`
- Passing-bay override: `rmf_platform-main/docker-compose.p4-passing-bay.yml`
- Portable MQTT override: `rmf_platform-main/docker-compose.portable.yml`
- Arbiter 실행: `scripts/run_direction_arbiter.sh`
- 개별 작업 전송: `scripts/t4_dispatch_via_arbiter.sh`

## 주요 디렉터리

```text
traffic_control/                 Direction Arbiter, Task Gate, Robot Tracker
config/                          Corridor, holding-bay, route 설정
tests/                           상태기계, fault safety, 구성 및 통합 테스트
scripts/                         setup/start/stop/dispatch 실행 도구
rmf_dev_tool-main/               VDA5050 Robot Simulator와 PyQt5 GUI
rmf_platform-main/               Compose, API 설정, Fleet Adapter, nav graph
docs/superpowers/specs/          설계 문서
docs/superpowers/plans/          구현 계획과 검증 항목
```

## DSR Lab 적용 전 하드닝 절차

DSR에서는 `simulation` 모드로 TaskGate를 띄우지 않는다. `lab` deployment
profile은 production과 같은 telemetry/robot readiness 검사를 사용하되,
실험실의 평문 MQTT(1883)를 위해 TLS와 인증 secret을 강제하지 않는다.

1. 실행 중인 `Wave_adapter`의 runtime navigation graph를 snapshot으로 복사한다.
2. `scripts/check_wave_runtime_graph.py`로 runtime graph와 snapshot fingerprint,
   corridor node/edge를 비교한다. 불일치하면 TaskGate를 시작하지 않는다.
3. `config/deployment.dsr-lab.example.yaml`을 복사해 현장값을 확인한 profile을
   만든다. 예제의 calibration/physical 값은 simulator 기준이므로 실물 투입
   전에 반드시 측정값으로 교체한다.
4. 아래 launcher로 시작한다. profile을 명시하지 않으면 실행을 거부한다.

```bash
EXPECTED_WAVE_NAV_REVISION=228 \
  ./scripts/start_dsr_lab_arbiter.sh /path/to/dsr-lab.yaml
```

MQTT `state`의 retained snapshot은 RobotTracker에 넣지 않는다. 주행 중
VDA5050 `edgeStates`가 존재하면 geometry보다 우선하며, 관리 대상이 아닌
side-branch edge를 broad corridor geometry로 재분류하지 않는다. 주행 중인
로봇은 `lastNodeId`가 Holding Bay여도 safe stop으로 취급하지 않는다.

`check_wave_runtime_graph.py`는 Holding Bay node에 관리되지 않은 incident
lane이 있으면 warning을 출력한다. DSR의 `1019`/`1024`처럼 junction 성격의
node를 Holding Bay로 사용할 때는 실제 대기 위치와 통행 간섭을 확인하고,
필요하면 본선 밖 staging node로 옮긴다.

Lab PoC 동안 managed task는 반드시 TaskGate
`http://127.0.0.1:18200/tasks/robot_task`로 보낸다. 기존 RMF API `:8100`으로
직접 제출하면 Arbiter를 우회하므로, UI/Robotpilot endpoint 전환 전에는
수동 실험 task만 18200을 사용한다.

## 실제 로봇 적용 전 주의

이 저장소의 JWT 생성은 로컬 시뮬레이션 전용이다. 실제 시스템에서는 현장 인증 서버의 service token을 사용해야 한다. 또한 아래 항목을 별도 검증해야 한다.

- 실제 좌표 측량과 holding bay 유효 폭
- 정지·위치 추정 오차 및 telemetry 누락
- 안전 PLC/EMS와의 연동
- 통신 단절 및 재기동 복구
- 실제 로봇 2v1/2v2, 다중 corridor, starvation 조건

telemetry가 release node를 놓치면 Arbiter는 fail-closed 상태를 유지하도록 설계되어 있다.

## 실차용 production profile

시뮬레이션 실행 파일과 별도로 `config/production.connected-corridor.example.yaml`을
현장 템플릿으로 사용한다. 예제의 `REPLACE_ME` 값, simulation 전용 map,
누락된 secret 또는 좌표 보정값은 의도적으로 preflight에 실패한다.

```bash
.venv/bin/python scripts/validate_production_deployment.py \
  config/production.connected-corridor.example.yaml
```

현장값을 모두 채운 뒤에는 Simulator를 시작하지 않는 launcher를 사용한다.
MQTT 비밀번호, 인증서 private key와 RMF token은 Git에 저장하지 않고 환경변수
또는 읽기 제한된 파일로 제공한다.

```bash
export FAB_MQTT_USER='<site mqtt user>'
export FAB_MQTT_PASSWORD='<site mqtt password>'
export RMF_API_TOKEN='<site service token>'
./scripts/start_production_corridor.sh /path/to/site-production.yaml

curl -sS http://127.0.0.1:8200/health
curl -sS http://127.0.0.1:8200/ready
```

초기 container build나 로봇 접속 시간이 더 필요하면
`PRODUCTION_HEALTH_TIMEOUT_SECONDS`(기본 300초)와
`PRODUCTION_READY_TIMEOUT_SECONDS`(기본 900초)를 조정할 수 있다.
launcher는 모든 required robot이 Fleet Adapter에 등록된 뒤 Arbiter를 시작한다.
기동이 실패하면 이 launcher가 올린 Adapter와 RMF 서비스들을 함께 중지해 8100
직접 endpoint가 반쯤 열린 상태로 남지 않게 한다.
production API server는 생성된 전용 설정으로 `127.0.0.1:8100`에만 bind하고
실제 시간을 사용한다. profile의 `rmf_api.url`도
`http://127.0.0.1:8100/tasks/robot_task`만 허용한다. 이 API token은 Task Gate
프로세스에만 제공하고 현장 작업 클라이언트에는 배포하지 않는다.

launcher는 profile의 Fleet config를 원본으로 삼되 MQTT/TLS, robot roster,
reference coordinates, footprint, 속도·가감속 값을 `.runtime/production-field`의
runtime-only 설정으로 합성한다. 같은 profile의 nav graph와 인증서를 컨테이너에
mount하므로 Arbiter와 Fleet Adapter가 서로 다른 현장 설정으로 뜨지 않는다.
preflight는 재구성 예제 맵, Fleet/profile robot 불일치, 존재하지 않는 charger,
Holding Bay/edge/release node와 nav graph 불일치도 거절한다. 좌표 보정은
`max_residual`, `min_scale`, `max_scale` 범위를 통과해야 하며, Block geometry를
가로지르는 모든 directed lane이 관리 edge에 포함되어야 한다.

`/health`는 프로세스 생존 여부이고 `/ready`는 MQTT/RMF 연결, 필수 로봇
telemetry와 SafeStop clean-start 조건을 모두 만족해 새 작업을 받아도 되는지를
나타낸다. 실제 적용 절차와 남은 제한은
`docs/RMF_VDA5050_Passing_Bay_PoC_Simulation_and_Field_Guide_2026-09-20.docx`에
정리되어 있다.

현재 production 경로는 단일 RMF level과 로봇당 하나의 VDA5050 map ID를
요구한다. RMF 작업은 반드시 8200 Task Gate로 제출하고 8100 RMF API를 현장
클라이언트에 직접 노출하지 않는다. Arbiter 상태는 메모리 기반이므로 통로 내부
재시작 후에는 자동 운행을 재개하지 않고 물리 위치 확인과 운영자 복구를 거친다.
좌표 보정을 사용하는 로봇은 작업 시작 시 graph에 존재하는 `lastNodeId`를
보고해야 한다. 값이 없으면 서로 다른 좌표계를 임의로 비교하지 않고 작업을
거부하므로, 현장 PLC/로봇의 VDA5050 State 계약에서 이를 먼저 확인한다.

## Git에 포함하지 않는 항목

- `.venv`, Python/ROS/Docker build cache
- 실행 로그, PID, `.runtime`
- SQLite DB와 API cache
- JWT, 비밀번호, `.env`
- Docker image binary

설계 상세:

- [Single Passing Bay 설계](docs/superpowers/specs/2026-09-16-p4-single-passing-bay-poc-design.md)
- [2106 무정지 조기 해제 설계](docs/superpowers/specs/2026-09-17-p4-nonstop-early-clear-design.md)
