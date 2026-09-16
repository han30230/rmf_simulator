# RMF + VDA5050 시뮬레이션 실행 가이드

## 1. 전체 구성

| 환경 | 구성요소 |
| --- | --- |
| WSL2 Ubuntu | Docker 기반 RMF Core, VDA5050 Fleet Adapter, Python Robot Simulator |
| Windows | `vda5050_gui.py` 로봇 경로 시각화, `rmf_graph_editor` 맵 편집 GUI |

RMF Core의 주요 서비스는 다음과 같다.

- `rmf_traffic_schedule`: 로봇 궤적 및 계획 관리
- `rmf_traffic_blockade`: 배타 구간 조정
- `rmf_task_dispatcher`: 태스크 분배
- `rmf_api_server`: REST/WebSocket API (`localhost:8100`)

Fleet Adapter와 Robot Simulator는 MQTT를 통해 VDA5050 메시지를 주고받는다.

## 2. 사전 준비: MQTT 브로커

`rmf_platform-main`과 `rmf_dev_tool-main`에는 MQTT 브로커가 포함되어 있지 않다. Fleet Adapter와 Robot Simulator 모두 `localhost:1883`을 사용하므로, 호스트 OS에 Mosquitto 등을 별도로 설치하고 실행해야 한다.

## 3. 실행 순서

### T1 — RMF Core 시작 (WSL2)

```bash
cd /mnt/d/Documents/ICS_code/rmf_platform-main/rmf_platform-main
sudo service docker start

docker compose up \
  rmf_traffic_schedule \
  rmf_traffic_blockade \
  rmf_task_dispatcher \
  rmf_api_server \
  -d
```

### T2 — Fleet Adapter 시작 (WSL2)

BEFORE 실행 (`penalty=0`):

```bash
cd /mnt/d/Documents/ICS_code/rmf_platform-main/rmf_platform-main

docker compose run --rm \
  -v "$(pwd)/fleet_config/p4_edit_before.yaml:/sim_config.yaml:ro" \
  -v "$(pwd)/map/p4_edit_node_add.yaml:/sim_map.yaml:ro" \
  vda5050_fleet_adapter \
  bash -c "cd /vda5050_ws && \
    rm -rf build/vda5050_fleet_adapter install/vda5050_fleet_adapter && \
    colcon build --packages-select vda5050_fleet_adapter && \
    source install/setup.bash && \
    ros2 run vda5050_fleet_adapter fleet_adapter \
      -c /sim_config.yaml -n /sim_map.yaml"
```

AFTER 실행 (`penalty=50`)은 `p4_edit_before.yaml` 대신 `p4_edit_after.yaml`을 마운트한다.

Corridor 설정 실행:

```bash
cd /mnt/d/Documents/ICS_code/rmf_platform-main/rmf_platform-main

docker compose run --rm \
  -v "$(pwd)/fleet_config/p4_edit_corridor_on.yaml:/sim_config.yaml:ro" \
  -v "$(pwd)/map/p4_edit_node_add.yaml:/sim_map.yaml:ro" \
  vda5050_fleet_adapter \
  bash -c "cd /vda5050_ws && \
    rm -rf build/vda5050_fleet_adapter install/vda5050_fleet_adapter && \
    colcon build --packages-select vda5050_fleet_adapter && \
    source install/setup.bash && \
    ros2 run vda5050_fleet_adapter fleet_adapter \
      -c /sim_config.yaml -n /sim_map.yaml"
```

### T3 — Robot Simulator 시작 (WSL2)

```bash
cd /mnt/d/Documents/ICS_code/rmf_dev_tool-main/rmf_dev_tool-main/vda5050_robot_simulator
~/sim_venv/bin/python run.py --config p4_scenario.yaml
```

### T4 — Task 발행 (WSL2)

Fleet Adapter 로그에서 Commission initialized 된 로봇 수를 확인한 뒤 실행한다.

```bash
for pair in \
  "AGV_A1 1602" "AGV_A2 1602" "AGV_A3 1602" \
  "AGV_B1 1599" "AGV_B2 1599" "AGV_B3 1599"; do
  robot=$(echo "$pair" | awk '{print $1}')
  dest=$(echo "$pair" | awk '{print $2}')

  curl -s --noproxy "*" \
    -X POST http://localhost:8100/tasks/robot_task \
    -H "Content-Type: application/json" \
    -d "{\"type\":\"robot_task_request\",\"robot\":\"${robot}\",\"fleet\":\"TOOL\",\"request\":{\"unix_millis_earliest_start_time\":0,\"category\":\"patrol\",\"priority\":{\"type\":\"default\",\"value\":0},\"description\":{\"places\":[\"${dest}\"],\"rounds\":1}}}" \
    2>/dev/null

  sleep 0.3
done
```

> 확인 필요: 위 명령은 목적지 `1599/1602`를 사용하지만 `p4_scenario.yaml` 주석은 `2101/2112`를 출발·목적지로 설명한다. 실제 사용 맵과 Fleet Adapter 설정에 맞는 번호를 사용해야 한다.

## 4. GUI 시각화 (Windows CMD)

```bat
D:
cd Documents\ICS_code\rmf_dev_tool-main\rmf_dev_tool-main\vda5050_gui
python vda5050_gui.py
```

GUI 설정:

1. `Mode` → `Live`
2. `MQTT` → `Connect`
3. Host: `127.0.0.1`
4. Port: `1883`
5. Prefix: `uagv/v2.0.0/inatech`
6. `File` → `Open` → `p4_edit_node_add.yaml`

실시간 MQTT 모드에서는 브로커 주소만 필요하다. 정적 로그 열기 모드까지 사용하려면 `vda5050_robot_simulator/logs/`도 함께 복사한다.

## 5. 결과 저장

```bash
FAID=$(docker ps --format "{{.ID}} {{.Names}}" | grep fleet | awk '{print $1}')
LABEL="before"  # 또는 after
RESULT_FILE="/mnt/d/Documents/ICS_code/${LABEL}_result.txt"

echo "=== ${LABEL} ===" | tee "$RESULT_FILE"
echo "negotiation: $(docker logs "$FAID" 2>&1 | grep -c 'negotiat')" \
  | tee -a "$RESULT_FILE"

docker logs "$FAID" 2>&1 | grep "3-tier path" | head -10 \
  | tee -a "$RESULT_FILE"
```

## 6. 맵 에디터 (`rmf_graph_editor`)

### 실행

```bat
D:
cd Documents\ICS_code\rmf_dev_tool-main\rmf_dev_tool-main\rmf_graph_editor
pip install pyyaml
python main.py
```

파일 열기: `Ctrl+O` → `p4_edit_node_add.yaml`

### 주요 조작

| 키/동작 | 기능 |
| --- | --- |
| `S` | 선택 모드, 노드·레인 선택 및 드래그 이동 |
| `N` | 노드 추가 |
| `E` | 엣지 추가: 출발 노드 → 도착 노드 |
| `D` | 선택 노드·레인 삭제 |
| `F` | 화면 전체 맞춤 |
| `Ctrl+Z` / `Ctrl+Y` | 실행 취소 / 다시 실행, 최대 50단계 |
| `Ctrl+S` | 저장 |
| 마우스 휠 | 확대·축소 |
| 더블클릭 | 전체 속성 편집 대화상자 |
| `Ctrl+O` | 파일 열기 |

### 양방향 Lane 추가

1. `S` 모드에서 기존 단방향 lane을 클릭하고 `start/end` 노드를 확인한다.
2. `E` 모드로 전환한다.
3. 기존 `end` 노드에서 `start` 노드 방향으로 반대 lane을 추가한다.
4. 오른쪽 패널에서 `speed_limit`, corridor 폭 등을 설정한다.
5. `Ctrl+S`로 저장한다.

### 주요 노드 속성

| 속성 | 설명 | 비고 |
| --- | --- | --- |
| `name` | Waypoint 이름 | Task 목적지 이름으로 사용 |
| `is_charger` | 충전기 여부 | Fleet 등록에 최소 1개 필요 |
| `is_holding_point` | 대기 가능 지점 | 로봇 대기 허용 |
| `mutex` | 동시 진입 제한 그룹 | 같은 그룹 lane의 배타 진입 |
| `narrow_corridor` | 좁은 복도 표시 | `CongestionAwareLaneCloser` 연동 |

### 주요 Lane 속성

| 속성 | 설명 |
| --- | --- |
| `speed_limit` | 최대 속도(m/s) |
| `rotationAllowed` | 구간 내 회전 허용 여부 |
| `corridor.leftWidth/rightWidth` | 복도 폭 및 충돌 감지 범위(m) |
| `mutex` | 동시 진입 제한 그룹명 |

### 노드 색상 범례

| 색상 | 의미 |
| --- | --- |
| 파란색 | `NONE`, 일반 waypoint |
| 노란색 | `CHGE`, 충전 스테이션 |
| 주황색 | `PICKDROP`, 픽업·드롭 위치 |
| 초록색 | `PARK`, 주차 스팟 |
| 보라 테두리 | `is_holding_point=true` |

## 7. 주요 설정 파일

| 파일 | 경로 | 용도 |
| --- | --- | --- |
| `p4_edit_node_add.yaml` | `rmf_platform-main/map/` | 시뮬레이션 맵 |
| `p4_edit_before.yaml` | `rmf_platform-main/fleet_config/` | BEFORE 설정, penalty=0 |
| `p4_edit_after.yaml` | `rmf_platform-main/fleet_config/` | AFTER 설정, penalty=50 |
| `p4_edit_corridor_on.yaml` | `rmf_platform-main/fleet_config/` | Corridor Manager 설정 |
| `p4_scenario.yaml` | `rmf_dev_tool-main/vda5050_robot_simulator/` | 로봇 초기 위치 및 시뮬레이터 설정 |
| `config.yaml` | `src/rmf_vda5050_fleet_adapter/vda5050_fleet_adapter/config/` | Fleet Adapter 설정 |

## 8. 이전 PC에서 옮겨야 할 구성

```text
rmf_platform-main/
├── docker-compose.yml
├── docker/Dockerfile
├── cyclonedds.xml
├── cyclonedds_rmf.xml
├── src/rmf_core/
├── src/rmf_vda5050_fleet_adapter/
├── src/rmf_battery_management/
├── src/rmf_commission_manager/
├── src/rmf_vda5050_rmf_bridge/
└── src/rmf_dev_tool/rmf_web_custom/

rmf_dev_tool-main/
├── vda5050_robot_simulator/
└── vda5050_gui/
```

### 현재 확인된 경로 문제

`docker-compose.yml`은 다음 맵을 참조한다.

```text
src/rmf_vda5050_fleet_adapter/map/map_dsr_0427.yaml
```

이 파일이 실제 워크스페이스에 없으면 `vda5050_fleet_adapter`, `battery_management`, `rmf_web_dashboard`의 volume mount 또는 실행이 실패한다. 반면 수동 T2 명령은 최상위의 `map/p4_edit_node_add.yaml`과 `fleet_config/p4_edit_corridor_on.yaml`을 사용한다. 어느 구조가 실제 기준인지 확인하고 경로를 통일해야 한다.
