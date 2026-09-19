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

```bash
./scripts/stop_p4_passing_bay.sh --all
./scripts/start_connected_corridor_chain.sh 1v3  # 또는 2v2
./scripts/launch_connected_corridor_chain_visualizer.sh

# 별도 터미널
./scripts/dispatch_connected_corridor_chain_1v3.sh  # 또는 ..._2v2.sh
```

중간 블록은 로봇이 다음 구간으로 넘어가면 순서대로 해제한다. 권한의 마지막
블록은 release node를 지나도 유지하며, 목적지 종점 또는 사이드 베이 안에
들어온 telemetry가 확인된 뒤에만 해제한다. 따라서 반대 방향 batch는 피신
로봇이 본선 junction에 도달한 시점이 아니라 실제 사이드 베이 도착 이후에
출발한다.

설정은 `config/corridor_blocks_connected_chain.yaml`, navigation graph는
`rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/connected_corridor_chain.yaml`
이다. 로봇 수, 로봇 이름, C1/C2/C3 node 이름은 production Python 정책에
포함되지 않으며, 현장 topology와 SafeStop/slot은 YAML로 정의한다.

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

## 실제 로봇 적용 전 주의

이 저장소의 JWT 생성은 로컬 시뮬레이션 전용이다. 실제 시스템에서는 현장 인증 서버의 service token을 사용해야 한다. 또한 아래 항목을 별도 검증해야 한다.

- 실제 좌표 측량과 holding bay 유효 폭
- 정지·위치 추정 오차 및 telemetry 누락
- 안전 PLC/EMS와의 연동
- 통신 단절 및 재기동 복구
- 실제 로봇 2v1/2v2, 다중 corridor, starvation 조건

telemetry가 release node를 놓치면 Arbiter는 fail-closed 상태를 유지하도록 설계되어 있다.

## Git에 포함하지 않는 항목

- `.venv`, Python/ROS/Docker build cache
- 실행 로그, PID, `.runtime`
- SQLite DB와 API cache
- JWT, 비밀번호, `.env`
- Docker image binary

설계 상세:

- [Single Passing Bay 설계](docs/superpowers/specs/2026-09-16-p4-single-passing-bay-poc-design.md)
- [2106 무정지 조기 해제 설계](docs/superpowers/specs/2026-09-17-p4-nonstop-early-clear-design.md)
