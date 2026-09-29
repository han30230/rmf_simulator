"""HTTP client for Adapter-level DSR admission.

This module intentionally depends only on the Python standard library so it can
be copied into the field Fleet Adapter without importing traffic_control.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from typing import Any, Sequence
from urllib import error as urllib_error
from urllib import request as urllib_request

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DsrAdmissionResult:
    decision: str
    managed: bool
    reason: str | None = None
    payload: dict[str, Any] | None = None

    @property
    def send_allowed(self) -> bool:
        return self.decision in {"ADMIT", "BYPASS"}


class DsrAdmissionClient:
    """Fail-closed client used immediately before VDA5050 order transmission."""

    def __init__(self, base_url: str, *, timeout: float = 0.25) -> None:
        normalized = str(base_url).strip().rstrip("/")
        if not normalized:
            raise ValueError("DSR admission base_url must not be empty")
        self.base_url = normalized
        self.timeout = float(timeout)
        if self.timeout <= 0.0:
            raise ValueError("DSR admission timeout must be positive")

    def admit(
        self,
        *,
        robot_id: str,
        movement_key: str,
        path: Sequence[str],
    ) -> DsrAdmissionResult:
        payload = {
            "robot_id": str(robot_id),
            "movement_key": str(movement_key),
            "path": [str(item) for item in path],
        }
        result = self._post("/traffic/adapter/admission", payload)
        if result is None:
            return DsrAdmissionResult(
                decision="BLOCKED",
                managed=True,
                reason="admission_service_unavailable",
            )

        decision = str(result.get("decision") or "BLOCKED").upper()
        if decision not in {"ADMIT", "WAIT", "BLOCKED", "BYPASS"}:
            logger.error(
                "Invalid DSR admission decision: robot=%s movement=%s result=%s",
                robot_id,
                movement_key,
                result,
            )
            return DsrAdmissionResult(
                decision="BLOCKED",
                managed=True,
                reason="invalid_admission_response",
                payload=result,
            )
        return DsrAdmissionResult(
            decision=decision,
            managed=bool(result.get("managed", decision != "BYPASS")),
            reason=(
                str(result["reason"]) if result.get("reason") is not None else None
            ),
            payload=result,
        )

    def cancel(self, *, robot_id: str, movement_key: str) -> bool:
        result = self._post(
            "/traffic/adapter/cancel",
            {
                "robot_id": str(robot_id),
                "movement_key": str(movement_key),
            },
        )
        return bool(result and result.get("cancelled"))

    def _post(
        self,
        path: str,
        payload: dict[str, Any],
    ) -> dict[str, Any] | None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request = urllib_request.Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib_request.urlopen(request, timeout=self.timeout) as response:
                decoded = json.loads(response.read().decode("utf-8"))
        except (
            urllib_error.HTTPError,
            urllib_error.URLError,
            TimeoutError,
            OSError,
            UnicodeDecodeError,
            json.JSONDecodeError,
        ) as error:
            logger.warning(
                "DSR admission request failed: endpoint=%s error=%s",
                path,
                error,
            )
            return None
        return decoded if isinstance(decoded, dict) else None
