# RMF + VDA5050 Traffic Simulation Workspace

집과 회사에서 재현한 RMF/VDA5050 시뮬레이션 작업공간이다. 현재 P4 단일 1차선 양방향 통로에서 Direction Arbiter가 로봇의 진입 방향, Block 점유, Holding Bay 예약, 대기 작업을 제어한다.

## 현재 확인된 범위

- Ubuntu 24.04 / ROS 2 Jazzy / Docker Desktop WSL2
- RMF Traffic Schedule, Task Dispatcher, Traffic Blockade, API Server
- VDA5050 Fleet Adapter와 AGV_A1, AGV_B1, AGV_B2 등록
- VDA5050 Robot Simulator 및 PyQt5 GUI
- RMF API Bearer 인증을 포함한 Arbiter Task 전달
- P4 `2101 ↔ 2108` 반대 방향 진입 제어
- A1 통과 후 B1/B2 동일 방향 동시 허가 및 전체 작업 완료
- 중앙 사이드 Passing Bay PoC 설계안 포함(아직 구현 전)

검증된 최종 상태에서는 A1이 `2101 → 2108`로 이동하는 동안 B1/B2가 대기하고, Block이 비워진 뒤 B1/B2가 `2108 → 2101`로 이동한다. 세 작업 모두 `COMPLETE`, Block은 `FREE`가 된다.

## 주요 구조

```text
.
├── traffic_control/                 # Direction Arbiter, Task Gate, Robot Tracker
├── config/                          # 기본/분할/P4 Corridor 설정
├── tests/                           # 교착, 점유, 고장, 인증, P4 구성 테스트
├── scripts/                         # Arbiter 실행 및 Task 전송 스크립트
├── rmf_dev_tool-main/
│   ├── vda5050_robot_simulator/     # 3대 로봇 MQTT 시뮬레이터
│   └── vda5050_gui/                 # 로그/맵/MQTT 모니터 GUI
├── rmf_platform-main/
│   ├── docker-compose.yml
│   ├── docker-compose.p4.yml        # P4 Map/Fleet 설정 override
│   └── src/rmf_vda5050_fleet_adapter/
└── docs/                            # 실행 가이드와 설계 문서
```

## 주의: WSL 배포판

PC에 `Ubuntu`와 `Ubuntu-24.04`가 함께 있으면 반드시 `Ubuntu-24.04`를 사용한다.

```powershell
wsl -d Ubuntu-24.04
```

```bash
echo "$WSL_DISTRO_NAME"
# Ubuntu-24.04
```

## 1. Python 환경 및 테스트

```bash
cd ~/rmf-work/rmf_simulation_workspace

python3 -m venv .venv
source .venv/bin/activate
python -m pip install fastapi uvicorn paho-mqtt pyyaml PyQt5

python -m unittest discover -s tests -p 'test_*.py' -v
```

## 2. RMF Core와 API Server

API Server 이미지 이름은 `rmf-api-server`, RMF Core는 `rmf-core:latest`를 사용한다.

```bash
cd ~/rmf-work/rmf_simulation_workspace/rmf_platform-main

docker compose up -d \
  rmf_traffic_schedule \
  rmf_task_dispatcher \
  rmf_traffic_blockade

docker compose up -d --force-recreate rmf_api_server

curl -i --noproxy '*' http://127.0.0.1:8100/
# HTTP 404이면 API 서버가 응답하는 상태
```

`sqlite_local_config.py`는 저장소에 포함되어 있으며 API를 `0.0.0.0:8100`에 연다.

## 3. VDA5050 Robot Simulator

```bash
cd ~/rmf-work/rmf_simulation_workspace/rmf_dev_tool-main/vda5050_robot_simulator
source ~/rmf-work/rmf_simulation_workspace/.venv/bin/activate

python run.py --config p4_scenario.yaml
```

정상 실행 시 AGV_A1은 2101, AGV_B1/B2는 2108 위치에서 MQTT에 연결된다.

## 4. P4 Fleet Adapter

```bash
cd ~/rmf-work/rmf_simulation_workspace/rmf_platform-main

docker compose \
  -f docker-compose.yml \
  -f docker-compose.p4.yml \
  up -d --force-recreate vda5050_fleet_adapter
```

등록 확인:

```bash
docker logs --tail 250 vda5050_fleet_adapter 2>&1 | \
grep -E 'Adding robot|Added a robot|Successfully added|ERROR|FATAL'
```

## 5. API 인증 토큰과 Direction Arbiter

현재 RMF API Server는 Bearer 인증을 요구한다. 개발용 JWT 생성과 실행 절차는 [RMF_API_AUTH_PATCH_README.md](RMF_API_AUTH_PATCH_README.md)를 따른다.

토큰을 같은 셸에 export한 뒤 Arbiter를 실행한다.

```bash
cd ~/rmf-work/rmf_simulation_workspace
source .venv/bin/activate

export RMF_API_BEARER_TOKEN='<development JWT>'

./scripts/run_direction_arbiter.sh \
  config/corridor_blocks_p4.yaml 2>&1 | tee arbiter_p4.log
```

상태 확인:

```bash
curl -s --noproxy '*' http://127.0.0.1:8200/traffic/status | python3 -m json.tool
```

## 6. T4 작업 전송

```bash
cd ~/rmf-work/rmf_simulation_workspace

./scripts/t4_dispatch_via_arbiter.sh AGV_A1 2108
sleep 1
./scripts/t4_dispatch_via_arbiter.sh AGV_B1 2101
./scripts/t4_dispatch_via_arbiter.sh AGV_B2 2101
```

예상 결과:

- A1: `ADMIT`
- B1/B2: 처음에는 `WAIT`
- A1 이탈 후 B1/B2: `TASK_RELEASED`
- 최종: 세 Job `COMPLETE`, `P4_CENTER_2101_2108` Block `FREE`

## 7. GUI

```bash
cd ~/rmf-work/rmf_simulation_workspace/rmf_dev_tool-main/vda5050_gui
source ~/rmf-work/rmf_simulation_workspace/.venv/bin/activate
python vda5050_gui.py
```

- 외부 `run.py`로 Simulator를 실행했다면 GUI의 **Monitor 탭**에서 로봇 3대를 확인한다.
- GUI의 **Simulation 탭**은 GUI 자체에서 Simulator를 시작할 때 사용한다.
- WSLg가 활성화된 `Ubuntu-24.04`에서 실행한다.

## 다음 작업

중앙 사이드 Holding Bay에 반대 방향 로봇이 잠시 빠졌다가 본선으로 복귀하는 PoC 설계는 아래 문서에 정리되어 있다.

- [P4 Single Passing Bay PoC 설계](docs/superpowers/specs/2026-09-16-p4-single-passing-bay-poc-design.md)

이 설계는 아직 구현 전이며, 다음 커밋에서 Map/Fleet/Corridor 설정과 자동 시나리오 테스트를 추가한다.

## 보안 및 저장소 제외 항목

가상환경, 로그, PID, DB, 캐시, 실제 JWT 및 `.env` 파일은 Git에 포함하지 않는다. 저장소에는 개발용 토큰 생성 방법만 포함한다.
