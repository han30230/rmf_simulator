"""VDA5050 Robot Simulator 진입점."""

import asyncio
import copy
import logging
import signal
import sys
import uuid
from datetime import date
from pathlib import Path

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, str(Path(__file__).parent))

import yaml

from vda5050_simulator.mqtt_client import MqttClient
from vda5050_simulator.robot import Robot
from vda5050_simulator.action_handler import ActionHandler
from vda5050_simulator.order_manager import OrderManager
from vda5050_simulator.state_publisher import StatePublisher
from vda5050_simulator.models import parse_instant_actions, Action, ActionParameter

_LOG_FORMAT = "%(asctime)s %(levelname)s: %(message)s"
_LOG_DATEFMT = "%Y-%m-%d %H:%M:%S"

logging.basicConfig(
    level=logging.INFO,
    format=_LOG_FORMAT,
    datefmt=_LOG_DATEFMT,
)
logger = logging.getLogger("vda5050_sim")


class _RobotLogFilter(logging.Filter):
    """특정 로봇 시리얼 번호가 포함된 로그만 통과시키는 필터."""

    def __init__(self, serial_number: str):
        super().__init__()
        self._tag = f"[{serial_number}]"

    def filter(self, record: logging.LogRecord) -> bool:
        return self._tag in record.getMessage()


def _setup_robot_file_handler(serial_number: str) -> logging.FileHandler:
    """로봇별 날짜 구분 로그 파일 핸들러를 생성하고 루트 로거에 등록."""
    log_dir = Path(__file__).parent / "logs" / serial_number
    log_dir.mkdir(parents=True, exist_ok=True)

    log_file = log_dir / f"{date.today().strftime('%Y-%m-%d')}.log"
    handler = logging.FileHandler(log_file, encoding="utf-8")
    handler.setLevel(logging.INFO)
    handler.setFormatter(logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATEFMT))
    handler.addFilter(_RobotLogFilter(serial_number))
    logging.getLogger().addHandler(handler)
    return handler


def _build_robot_config(mqtt_cfg: dict, pub_cfg: dict, defaults: dict, robot_entry: dict) -> dict:
    """robot_defaults와 개별 로봇 설정을 merge하여 단일 로봇 config를 생성."""
    robot_cfg = copy.deepcopy(defaults)
    for key, value in robot_entry.items():
        if isinstance(value, dict) and key in robot_cfg and isinstance(robot_cfg[key], dict):
            robot_cfg[key].update(value)
        else:
            robot_cfg[key] = value
    return {
        "mqtt": mqtt_cfg,
        "robot": robot_cfg,
        "publishing": pub_cfg,
    }


class Simulator:
    def __init__(self, config: dict):
        self._config = config
        self._loop: asyncio.AbstractEventLoop | None = None
        self._robot = Robot(config)
        self._order_manager = OrderManager(self._robot)
        self._action_handler = ActionHandler(
            self._robot, self._order_manager,
            action_results=config.get("action_results", {}),
        )
        self._robot.set_action_handler(self._action_handler)
        self._mqtt = MqttClient(config)
        self._state_publisher = StatePublisher(self._robot, self._mqtt, config)

        self._download_map_cfg = config.get("download_map")

        # MQTT 콜백 등록 (paho-mqtt 스레드에서 호출됨)
        self._mqtt.set_callbacks(
            on_order=self._handle_order,
            on_instant_actions=self._handle_instant_actions,
            on_connection_online=self._handle_connection_online,
        )

        self._serial = config["robot"]["serial_number"]
        self._cancel_in_progress: asyncio.Event | None = None  # cancelOrder 진행 중 대기용

    @property
    def robot(self) -> Robot:
        return self._robot

    def set_peer_robots(self, robots: list[Robot]):
        """같은 프로세스에서 실행 중인 로봇 목록을 공유한다."""
        self._robot.set_peer_robots(robots)

    def _handle_order(self, payload: dict):
        """Order 메시지 수신 콜백. (paho-mqtt 스레드에서 호출)."""
        if self._loop:
            self._loop.call_soon_threadsafe(
                lambda p=payload: asyncio.ensure_future(self._process_order_async(p)),
            )

    async def _process_order_async(self, payload: dict):
        """Order 처리. cancelOrder 진행 중이면 완료될 때까지 대기."""
        if self._cancel_in_progress is not None:
            logger.info("[%s] cancelOrder 진행 중 — 완료 대기 후 주문 처리", self._serial)
            await self._cancel_in_progress.wait()
            logger.info("[%s] cancelOrder 완료 — 주문 처리 재개", self._serial)
        success = self._order_manager.process_order(payload)
        if success:
            logger.info("[%s] 주문 수락 완료", self._serial)
        else:
            logger.warning("[%s] 주문 거부됨", self._serial)
        self._state_publisher.publish_state_now()

    def _handle_connection_online(self):
        """ONLINE 연결 콜백. downloadMap 전송. (paho-mqtt 스레드에서 호출)."""
        if self._loop and self._download_map_cfg:
            self._loop.call_soon_threadsafe(self._trigger_download_map)

    def _trigger_download_map(self):
        """downloadMap instantAction 생성 및 실행."""
        cfg = self._download_map_cfg
        action_id = f"downloadMap_{uuid.uuid4().hex[:8]}"
        action = Action(
            actionType="downloadMap",
            actionId=action_id,
            blockingType="NONE",
            actionParameters=[
                ActionParameter(key="mapId", value=cfg["map_id"]),
                ActionParameter(key="mapDownloadUrl", value=cfg["map_download_url"]),
                ActionParameter(key="mapVersion", value=cfg["map_version"]),
            ],
        )
        self._robot.set_download_map_pending(action_id)
        logger.info(
            "[%s] downloadMap 전송: mapId=%s, version=%s",
            self._serial, cfg["map_id"], cfg["map_version"],
        )
        asyncio.ensure_future(self._action_handler.execute_instant_action(action))

    def _handle_instant_actions(self, payload: dict):
        """InstantActions 메시지 수신 콜백. (paho-mqtt 스레드에서 호출)."""
        if self._loop:
            self._loop.call_soon_threadsafe(self._process_instant_actions, payload)

    def _process_instant_actions(self, payload: dict):
        """InstantActions 처리. (asyncio 스레드에서 실행)."""
        try:
            ia = parse_instant_actions(payload)
        except (KeyError, TypeError, ValueError) as e:
            logger.error("[%s] InstantActions 파싱 실패: %s", self._serial, e)
            return

        for action in ia.actions:
            logger.info("[%s] InstantAction 수신: %s (id=%s)", self._serial, action.actionType, action.actionId)
            if action.actionType == "cancelOrder":
                asyncio.ensure_future(self._execute_cancel_order(action))
            else:
                asyncio.ensure_future(self._action_handler.execute_instant_action(action))

    async def _execute_cancel_order(self, action):
        """cancelOrder를 실행하고, 완료될 때까지 새 주문 처리를 블로킹."""
        self._cancel_in_progress = asyncio.Event()
        try:
            await self._action_handler.execute_instant_action(action)
        finally:
            self._cancel_in_progress.set()
            self._cancel_in_progress = None
            logger.info("[%s] cancelOrder 완료 — 새 주문 수락 가능", self._serial)

    async def run(self, stop_event: asyncio.Event):
        """시뮬레이터 실행. stop_event가 set되면 종료."""
        self._loop = asyncio.get_running_loop()
        self._robot.set_loop(self._loop)
        self._state_publisher.set_loop(self._loop)

        robot_cfg = self._config["robot"]
        logger.info("=" * 60)
        logger.info("[%s] VDA5050 Robot Simulator 시작", self._serial)
        logger.info("  제조사: %s", robot_cfg["manufacturer"])
        logger.info("  시리얼: %s", robot_cfg["serial_number"])
        logger.info("  초기 위치: (%.1f, %.1f)", robot_cfg["initial_position"]["x"],
                     robot_cfg["initial_position"]["y"])
        logger.info("=" * 60)

        # MQTT 연결
        try:
            self._mqtt.connect()
        except Exception as e:
            logger.error("[%s] MQTT 브로커 연결 실패: %s", self._serial, e)
            logger.error("mosquitto가 실행 중인지 확인하세요")
            return

        # 초기 State 즉시 발행
        await asyncio.sleep(0.5)
        self._state_publisher.publish_state_now()

        # 비동기 태스크 시작
        tasks = [
            asyncio.create_task(self._robot.navigation_loop()),
            asyncio.create_task(self._robot.charging_loop()),
            asyncio.create_task(self._state_publisher.state_publish_loop()),
            asyncio.create_task(self._state_publisher.visualization_publish_loop()),
        ]

        await stop_event.wait()

        # 정리
        logger.info("[%s] 시뮬레이터 종료 중...", self._serial)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

        self._mqtt.disconnect()
        logger.info("[%s] 시뮬레이터 종료 완료", self._serial)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml", help="config yaml file")
    parser.add_argument(
        "--fault-scenarios",
        help="optional YAML file containing fault_injection.enabled/rules",
    )
    args, _ = parser.parse_known_args()

    config_path = Path(__file__).parent / args.config
    if not config_path.is_absolute():
        config_path = Path(__file__).parent / args.config
    if not config_path.exists():
        logger.error("config 파일을 찾을 수 없습니다: %s", config_path)
        sys.exit(1)

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    fault_rules = []
    if args.fault_scenarios:
        fault_path = Path(args.fault_scenarios)
        if not fault_path.is_absolute():
            fault_path = Path(__file__).parent / fault_path
        with open(fault_path, "r", encoding="utf-8") as f:
            fault_config = yaml.safe_load(f) or {}
        injection = fault_config.get("fault_injection", {})
        if injection.get("enabled", False):
            fault_rules = injection.get("rules", [])
    else:
        injection = config.get("fault_injection", {})
        if injection.get("enabled", False):
            fault_rules = injection.get("rules", [])

    mqtt_cfg = config["mqtt"]
    pub_cfg = config.get("publishing", {
        "state_interval": 0.5,
        "visualization_interval": 0.5,
        "simulation_tick": 0.05,
    })
    defaults = config.get("robot_defaults", {})
    robots = config.get("robots", [])
    download_map_cfg = config.get("download_map")
    action_results_cfg = config.get("action_results", {})

    if not robots:
        logger.error("robots 목록이 비어 있습니다. config.yaml을 확인하세요.")
        sys.exit(1)

    # 각 로봇별 Simulator 인스턴스 생성 및 파일 로거 설정
    simulators = []
    for robot_entry in robots:
        robot_config = _build_robot_config(mqtt_cfg, pub_cfg, defaults, robot_entry)
        if download_map_cfg:
            robot_config["download_map"] = download_map_cfg
        if action_results_cfg:
            robot_config["action_results"] = action_results_cfg
        robot_config["fault_injection"] = fault_rules
        serial = robot_entry.get("serial_number", "UNKNOWN")
        _setup_robot_file_handler(serial)
        simulators.append(Simulator(robot_config))

    all_robots = [sim.robot for sim in simulators]
    for sim in simulators:
        sim.set_peer_robots(all_robots)

    logger.info("총 %d대의 로봇 시뮬레이터를 시작합니다.", len(simulators))

    async def run_all():
        stop_event = asyncio.Event()
        loop = asyncio.get_running_loop()

        def _signal_handler(*_):
            logger.info("종료 시그널 수신")
            loop.call_soon_threadsafe(stop_event.set)

        try:
            # Unix/Linux 전용 asyncio signal handler
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, _signal_handler)
        except NotImplementedError:
            # Windows: signal.signal 폴백 (Ctrl+C만 처리)
            signal.signal(signal.SIGINT, _signal_handler)
            if hasattr(signal, "SIGTERM"):
                signal.signal(signal.SIGTERM, _signal_handler)

        await asyncio.gather(*(sim.run(stop_event) for sim in simulators))

    asyncio.run(run_all())


if __name__ == "__main__":
    main()
