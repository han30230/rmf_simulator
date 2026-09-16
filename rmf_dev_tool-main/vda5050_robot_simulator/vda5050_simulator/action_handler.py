"""Action 생명주기 관리."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

from .models import Action, ActionState, Load, MapState

if TYPE_CHECKING:
    from .robot import Robot
    from .order_manager import OrderManager

logger = logging.getLogger(__name__)

# 액션 시뮬레이션 소요 시간 (초)
ACTION_DURATIONS = {
    "pick": 0.0,       # 핸들러에서 모션 직접 처리
    "drop": 0.0,       # 핸들러에서 모션 직접 처리
    "startCharging": 0.0,  # 핸들러에서 모션 직접 처리
    "stopCharging": 0.0,   # 핸들러에서 모션 직접 처리
    "initPosition": 0.5,
    "finePositioning": 2.0,
    "startPause": 0.0,  # 즉시 완료
    "stopPause": 0.0,
    "cancelOrder": 0.0,
    "factsheetRequest": 0.0,
    "downloadMap": 3.0,  # 맵 다운로드 시뮬레이션
}


class ActionHandler:
    def __init__(self, robot: Robot, order_manager: OrderManager | None = None, action_results: dict | None = None):
        self._robot = robot
        self._order_manager = order_manager
        self._action_results = action_results or {}
        self._running_tasks: dict[str, asyncio.Task] = {}
        self._log_prefix = f"[{robot._serial_number}]"

    def create_action_state(self, action: Action, initial_status: str = "WAITING") -> ActionState:
        return ActionState(
            actionId=action.actionId,
            actionType=action.actionType,
            actionStatus=initial_status,
            actionDescription=action.actionDescription,
        )

    async def execute_instant_action(self, action: Action):
        """InstantAction으로 수신된 액션을 즉시 실행."""
        action_state = self.create_action_state(action, "INITIALIZING")
        self._robot.action_states.append(action_state)
        self._robot.notify_state_change()

        await self._run_action(action, action_state)

    async def trigger_action(self, action: Action, action_state: ActionState):
        """노드/엣지 도달 시 대기 중이던 액션을 실행."""
        if action_state.actionStatus != "WAITING":
            return
        action_state.actionStatus = "INITIALIZING"
        self._robot.notify_state_change()

        task = asyncio.create_task(self._run_action(action, action_state))
        self._running_tasks[action.actionId] = task

    async def _run_action(self, action: Action, action_state: ActionState):
        action_type = action.actionType
        logger.info("%s 액션 실행: %s (id=%s, blocking=%s)", self._log_prefix, action_type, action.actionId, action.blockingType)

        action_state.actionStatus = "RUNNING"
        self._robot.notify_state_change()

        try:
            if action_type == "startPause":
                self._robot.paused = True
                logger.info("%s 일시정지 활성화", self._log_prefix)

            elif action_type == "stopPause":
                self._robot.paused = False
                logger.info("%s 일시정지 해제", self._log_prefix)

            elif action_type == "cancelOrder":
                await self._handle_cancel_order(action_state)
                return  # cancelOrder는 자체적으로 FINISHED 처리

            elif action_type == "startCharging":
                await self._handle_start_charging(action)

            elif action_type == "stopCharging":
                await self._handle_stop_charging(action)

            elif action_type == "pick":
                await self._handle_pick(action)

            elif action_type == "drop":
                await self._handle_drop(action)

            elif action_type == "initPosition":
                self._handle_init_position(action)

            elif action_type == "downloadMap":
                self._robot.set_download_map_pending(action.actionId)
                self._handle_download_map(action)

            else:
                # 알 수 없는 액션은 일정 시간 후 완료
                logger.info("%s 일반 액션 실행: %s", self._log_prefix, action_type)

            duration = ACTION_DURATIONS.get(action_type, 2.0)
            if duration > 0:
                await asyncio.sleep(duration)

            # config의 action_results 설정에 따라 성공/실패 결정
            configured_result = self._action_results.get(action_type, "FINISHED")
            if configured_result == "FAILED":
                action_state.actionStatus = "FAILED"
                action_state.resultDescription = f"{action_type} failed (configured)"
                logger.warning("%s 액션 실패 (설정): %s (id=%s)", self._log_prefix, action_type, action.actionId)
            else:
                action_state.actionStatus = "FINISHED"
                action_state.resultDescription = f"{action_type} completed"
                logger.info("%s 액션 완료: %s (id=%s)", self._log_prefix, action_type, action.actionId)

        except asyncio.CancelledError:
            action_state.actionStatus = "FAILED"
            action_state.resultDescription = "Action cancelled"
            logger.info("%s 액션 취소됨: %s (id=%s)", self._log_prefix, action_type, action.actionId)

        finally:
            self._running_tasks.pop(action.actionId, None)
            self._robot.notify_state_change()

    async def _handle_cancel_order(self, action_state: ActionState):
        """주문 취소 처리."""
        logger.info("%s 주문 취소 처리 시작", self._log_prefix)

        # 실행 중인 다른 액션을 취소
        for task_id, task in list(self._running_tasks.items()):
            if task_id != action_state.actionId:
                task.cancel()

        # 대기 중인 액션을 FAILED로 변경
        for astate in self._robot.action_states:
            if astate.actionId == action_state.actionId:
                continue
            if astate.actionStatus in ("WAITING", "INITIALIZING", "RUNNING"):
                astate.actionStatus = "FAILED"
                astate.resultDescription = "Cancelled by cancelOrder"

        # 이동 중지
        self._robot.cancel_current_order()

        # OrderManager의 현재 주문도 클리어
        if self._order_manager:
            self._order_manager.clear_order()

        action_state.actionStatus = "FINISHED"
        action_state.resultDescription = "Order cancelled"
        self._robot.notify_state_change()
        logger.info("%s 주문 취소 완료", self._log_prefix)

    async def _handle_pick(self, action: Action):
        """화물 적재 시뮬레이션: 1500mm 후진 → 1500mm 전진."""
        load_type = None
        for p in action.actionParameters:
            if p.key == "loadType":
                load_type = p.value
        new_load = Load(
            loadId=f"load-{action.actionId[:8]}",
            loadType=load_type or "UNKNOWN",
        )
        self._robot.loads.append(new_load)
        logger.info("%s pick: 화물 적재 시작 (loadType=%s) — 후진 1500mm", self._log_prefix, new_load.loadType)

        await self._robot.move_linear(1.5, forward=False)
        logger.info("%s pick: 후진 1500mm 완료, 5초 대기 (위치=(%.2f, %.2f))", self._log_prefix, self._robot.position_x, self._robot.position_y)
        await asyncio.sleep(5.0)
        await self._robot.move_linear(1.5, forward=True)
        logger.info("%s pick: 전진 1500mm 완료, 적재 완료 (위치=(%.2f, %.2f))", self._log_prefix, self._robot.position_x, self._robot.position_y)

    async def _handle_drop(self, action: Action):
        """화물 하역 시뮬레이션: 1500mm 후진 → 5초 대기 → 1500mm 전진."""
        if self._robot.loads:
            removed = self._robot.loads.pop(0)
            logger.info("%s drop: 화물 하역 시작 (loadType=%s) — 후진 1500mm", self._log_prefix, removed.loadType)
        else:
            logger.warning("%s drop: 하역할 화물 없음", self._log_prefix)

        await self._robot.move_linear(1.5, forward=False)
        logger.info("%s drop: 후진 1500mm 완료, 5초 대기 (위치=(%.2f, %.2f))", self._log_prefix, self._robot.position_x, self._robot.position_y)
        await asyncio.sleep(5.0)
        await self._robot.move_linear(1.5, forward=True)
        logger.info("%s drop: 전진 1500mm 완료, 하역 완료 (위치=(%.2f, %.2f))", self._log_prefix, self._robot.position_x, self._robot.position_y)

    async def _handle_start_charging(self, action: Action):
        """충전 시작 시뮬레이션: 500mm 전진 후 충전 시작."""
        logger.info("%s 충전 시작 — 전진 500mm", self._log_prefix)
        await self._robot.move_linear(0.5, forward=True)
        self._robot.battery_state_charging = True
        logger.info("%s 충전 도킹 완료, 충전 시작 (배터리=%.1f%%)", self._log_prefix, self._robot._battery_charge)

    async def _handle_stop_charging(self, action: Action):
        """충전 중지 시뮬레이션: 충전 중지 후 500mm 후진."""
        self._robot.battery_state_charging = False
        logger.info("%s 충전 중지 — 후진 500mm (배터리=%.1f%%)", self._log_prefix, self._robot._battery_charge)
        await self._robot.move_linear(0.5, forward=False)
        logger.info("%s 충전 언도킹 완료", self._log_prefix)

    def _handle_init_position(self, action: Action):
        """위치 초기화."""
        x, y, theta, map_id = 0.0, 0.0, 0.0, ""
        for p in action.actionParameters:
            if p.key == "x":
                x = float(p.value)
            elif p.key == "y":
                y = float(p.value)
            elif p.key == "theta":
                theta = float(p.value)
            elif p.key == "mapId":
                map_id = p.value

        self._robot.position_x = x
        self._robot.position_y = y
        self._robot.position_theta = theta
        if map_id:
            self._robot.map_id = map_id
        self._robot.position_initialized = True
        logger.info("%s 위치 초기화: (%.2f, %.2f, %.2f) map=%s", self._log_prefix, x, y, theta, map_id)

    def _handle_download_map(self, action: Action):
        """맵 다운로드 시뮬레이션."""
        map_id = ""
        map_download_url = ""
        map_version = ""
        for p in action.actionParameters:
            if p.key == "mapId":
                map_id = p.value
            elif p.key == "mapDownloadUrl":
                map_download_url = p.value
            elif p.key == "mapVersion":
                map_version = p.value

        logger.info("%s 맵 다운로드 중: mapId=%s, version=%s, url=%s", self._log_prefix, map_id, map_version, map_download_url)

        # robot.maps 업데이트 (기존 mapId가 있으면 업데이트, 없으면 추가)
        updated = False
        for m in self._robot.maps:
            if m.mapId == map_id:
                m.mapVersion = map_version
                m.mapStatus = "ENABLED"
                updated = True
                break
        if not updated:
            self._robot.maps.append(
                MapState(mapId=map_id, mapVersion=map_version, mapStatus="ENABLED")
            )
        logger.info("%s 맵 다운로드 완료: mapId=%s, version=%s", self._log_prefix, map_id, map_version)

    def has_blocking_action(self) -> bool:
        """HARD 블로킹 액션이 실행 중인지 확인."""
        for astate in self._robot.action_states:
            if astate.actionStatus in ("INITIALIZING", "RUNNING"):
                # 해당 액션의 blockingType 확인
                for action_id, task in self._running_tasks.items():
                    if action_id == astate.actionId:
                        return True  # 실행 중인 액션이 있으면 일단 true
        return False

    async def cancel_all(self):
        """모든 실행 중인 액션 취소."""
        for task in list(self._running_tasks.values()):
            task.cancel()
        self._running_tasks.clear()
