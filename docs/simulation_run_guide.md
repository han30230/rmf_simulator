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
B2의 `2108 → 6137` 구간은 A1의 HB_RIGHT 목적지 예약이 해제될 때까지
보류하여 A1의 2106–2108 잔여 경로와 겹치지 않게 한다.

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
5. A1은 2106에서 정지하지 않고 주행하며, B1은 `6137 → 2101`로 출발한다.
6. 목적지 holding-bay 예약은 각 로봇이 실제 도착할 때 해제된다.

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
