# Field Deployment Readiness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add fail-closed production configuration, secured MQTT, VDA5050 robot eligibility/readiness, and configurable simulator fault injection while preserving existing corridor simulations.

**Architecture:** A top-level deployment profile is validated before production processes start. Geometry occupancy remains in `RobotTracker`, while a pure eligibility policy combines VDA5050 state and connection data and Task Gate checks runtime readiness before every rolling authority. Fleet Adapter and Arbiter keep independent MQTT clients with identical security semantics; simulator fault injection emits ordinary VDA5050 messages.

**Tech Stack:** Python 3, dataclasses, PyYAML, paho-mqtt, FastAPI, unittest, Bash, Docker Compose, Open-RMF, VDA5050 2.0.0

**Spec:** `docs/superpowers/specs/2026-09-20-field-deployment-readiness-design.md`

## Global Constraints

- Do not modify RMF Core Planner.
- Do not hardcode production robot IDs, node IDs, credentials, or current scenario values in production Python.
- Preserve all existing simulation profiles and normal scenarios.
- Reject incomplete production map, calibration, physical limits, identity, MQTT security, or RMF credentials before motion services start.
- Resolve secrets from environment variables or files; never expose them in Git, logs, status, or errors.
- Production TLS always verifies the broker.
- Arbiter may become ready automatically only when every required robot is freshly observed at a configured SafeStop and every managed Block is empty.
- Use TDD and observe each new test fail for the intended reason before production changes.
- Never stage or modify `.runtime`, `recording_260919.jsonl`, the generated DOCX, its Word lock file, certificates, keys, or token files.

## File Structure

- `traffic_control/deployment.py`: typed profile, secret references, validation, redaction.
- `traffic_control/eligibility.py`: pure robot operational eligibility policy.
- `traffic_control/readiness.py`: process/dependency/clean-start readiness.
- `traffic_control/robot_tracker.py`: VDA5050 state and connection ingestion plus secured MQTT monitor.
- `traffic_control/task_gate.py`: admission checks and health/readiness endpoints.
- Fleet Adapter `config_port.py`, `main.py`, `mqtt_client.py`: authenticated TLS MQTT.
- Simulator `fault_injection.py`: config-driven triggers and VDA5050 fault effects.
- Production preflight/launcher scripts and field template.

---

### Task 1: Deployment Profile and Production Preflight

**Files:**
- Create: `traffic_control/deployment.py`
- Create: `tests/test_deployment_config.py`
- Create: `config/production.connected-corridor.example.yaml`
- Create: `scripts/validate_production_deployment.py`

**Interfaces:**
- Produces: `DeploymentProfile.load(path: Path, environ: Mapping[str, str] | None = None) -> DeploymentProfile`
- Produces: `DeploymentProfile.validate() -> tuple[str, ...]`
- Produces: `DeploymentProfile.require_valid() -> None`
- Produces: `DeploymentProfile.redacted_snapshot() -> dict[str, Any]`
- Produces: `SecretRef.resolve(environ: Mapping[str, str]) -> str`
- Produces: `MqttDeploymentConfig(host, port, keepalive_sec, reconnect_max_delay_sec, username, password, ca_file, cert_file, key_file, tls_required)`
- Produces: `RobotDeploymentConfig(manufacturer, serial_number, allowed_map_ids, required)`

- [ ] **Step 1: Write failing validation tests**

Cover simulation acceptance and production rejection for placeholders, `simulation_only`, duplicate robot identity, missing allowed map, fewer than three or collinear coordinate pairs, nonpositive physical/timing data, unresolved secrets, missing CA/cert/key, insecure TLS, and missing RMF token.

```python
def test_production_rejects_placeholder_and_simulation_map(self):
    profile = DeploymentProfile.load(
        write_profile(self, broker="REPLACE_ME", simulation_only=True),
        environ={},
    )
    self.assertIn("mqtt.host.placeholder", profile.validate())
    self.assertIn("map.simulation_only", profile.validate())

def test_redacted_snapshot_omits_values(self):
    profile = DeploymentProfile.load(valid_profile(self), environ={
        "FAB_MQTT_PASSWORD": "super-secret", "RMF_API_TOKEN": "signed-token",
    })
    rendered = json.dumps(profile.redacted_snapshot())
    self.assertNotIn("super-secret", rendered)
    self.assertNotIn("signed-token", rendered)
```

- [ ] **Step 2: Run RED**

Run: `.venv/bin/python -m unittest tests.test_deployment_config -v`

Expected: FAIL with missing `traffic_control.deployment`.

- [ ] **Step 3: Implement typed loading, validation, and redaction**

Use frozen dataclasses, resolve relative paths from the profile directory, and return stable dotted error codes. `SecretRef` supports exactly one of `env` or `file`.

```python
@dataclass(frozen=True)
class SecretRef:
    env: str | None = None
    file: Path | None = None

    def resolve(self, environ: Mapping[str, str]) -> str:
        value = environ.get(self.env, "") if self.env else (
            self.file.read_text(encoding="utf-8").strip() if self.file else ""
        )
        if not value:
            raise DeploymentConfigError("secret.unresolved")
        return value
```

- [ ] **Step 4: Add the intentionally incomplete field template and CLI**

The CLI prints `ERROR <stable-code>` per error, returns `2` when invalid and `0` when valid. The committed example contains replacement markers and environment names, never credentials.

```python
errors = DeploymentProfile.load(Path(args.profile)).validate()
for code in errors:
    print(f"ERROR {code}", file=sys.stderr)
return 2 if errors else 0
```

- [ ] **Step 5: Verify and commit**

Run: `.venv/bin/python -m unittest tests.test_deployment_config tests.test_config_files tests.test_connected_corridor_chain_config -v`

Expected: PASS. Then commit only the four Task 1 files with message `Add production deployment preflight`.

### Task 2: Secured MQTT in Fleet Adapter and Arbiter

**Files:**
- Modify: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/vda5050_fleet_adapter/usecase/ports/config_port.py`
- Modify: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/vda5050_fleet_adapter/presentation/main.py`
- Modify: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/vda5050_fleet_adapter/infra/mqtt/mqtt_client.py`
- Modify: `traffic_control/robot_tracker.py`
- Create: `tests/test_vda5050_mqtt_security.py`
- Create: `tests/test_arbiter_mqtt_security.py`

**Interfaces:**
- Consumes: validated field meanings from `MqttDeploymentConfig`; the Fleet Adapter does not import the top-level `traffic_control` package because it runs in a separate ROS container.
- Produces: adapter `MqttConfig` fields `username`, `password`, `ca_file`, `cert_file`, `key_file`, `tls_required`.
- Produces: `MqttStateMonitor(..., mqtt_config: MqttDeploymentConfig | None = None)` and `is_connected: bool`.

- [ ] **Step 1: Write failing client tests**

Patch `paho.mqtt.client.Client` and assert both clients call `username_pw_set`, `tls_set`, never `tls_insecure_set`, expose connection state, and subscribe to state and connection topics.

```python
client.username_pw_set.assert_called_once_with("operator", "secret")
client.tls_set.assert_called_once_with(
    ca_certs="/certs/ca.pem",
    certfile="/certs/client.pem",
    keyfile="/certs/client.key",
)
client.tls_insecure_set.assert_not_called()
```

- [ ] **Step 2: Run RED**

Run: `.venv/bin/python -m unittest tests.test_vda5050_mqtt_security tests.test_arbiter_mqtt_security -v`

Expected: FAIL because security fields and connection subscription do not exist.

- [ ] **Step 3: Implement shared MQTT security semantics**

Configure credentials and verified TLS before connect; require cert/key together; update `_connected` in callbacks. The production launcher passes the validated environment references to both processes. Fleet Adapter resolves `fleet_manager.security.username_env`, `password_env`, `ca_file`, `client_cert_file`, and `client_key_file` without importing `traffic_control`; Arbiter receives the typed deployment object. Error messages may name a missing environment variable but never contain a resolved value.

- [ ] **Step 4: Verify and commit**

Run: `.venv/bin/python -m unittest tests.test_vda5050_mqtt_security tests.test_arbiter_mqtt_security -v`

Expected: PASS. Commit Task 2 files with message `Support authenticated TLS MQTT connections`.

### Task 3: Robot Operational Eligibility

**Files:**
- Create: `traffic_control/eligibility.py`
- Modify: `traffic_control/robot_tracker.py`
- Modify: `traffic_control/task_gate.py`
- Create: `tests/test_robot_eligibility.py`
- Modify: `tests/test_fault_safety.py`
- Modify: `tests/test_corridor_chain_task_gate.py`

**Interfaces:**
- Produces: `EligibilityResult(eligible: bool, reasons: tuple[str, ...])`.
- Produces: `RobotEligibilityPolicy.evaluate(telemetry: RobotTelemetry | None, now: float) -> EligibilityResult`.
- Produces: `RobotTracker.ingest_connection(robot_id, payload, received_at=None) -> None`.
- Produces: `RobotTracker.eligibility(robot_id, now=None) -> EligibilityResult`.

- [ ] **Step 1: Write a failing eligibility table**

Cover unknown robot, offline/stale connection, stale state, E-stop, field violation, MANUAL, paused, blocking error, wrong map, uninitialized position, and healthy automatic operation.

```python
cases = {
    "offline": ({"connection_state": "OFFLINE"}, "connection.offline"),
    "estop": ({"e_stop": "AUTOACK"}, "safety.estop"),
    "manual": ({"operating_mode": "MANUAL"}, "mode.not_automatic"),
    "wrong_map": ({"map_id": "OTHER"}, "position.map_mismatch"),
}
for name, (changes, reason) in cases.items():
    with self.subTest(name=name):
        result = policy.evaluate(replace(healthy(), **changes), now=100.0)
        self.assertFalse(result.eligible)
        self.assertIn(reason, result.reasons)
```

- [ ] **Step 2: Write failing parsing and ordering tests**

Parse header identity/ID/timestamp, position initialization/map ID, mode, safety, paused and errors. Ignore lower header IDs per robot/topic and reject payload manufacturer/serial mismatch.

- [ ] **Step 3: Run RED**

Run: `.venv/bin/python -m unittest tests.test_robot_eligibility tests.test_fault_safety tests.test_corridor_chain_task_gate -v`

Expected: new tests FAIL; existing tests remain PASS.

- [ ] **Step 4: Implement the pure policy and telemetry ingestion**

Keep geometry classification unchanged and call small operational parsing helpers. Production defaults to blocking VDA5050 `FATAL` errors and may be configured stricter.

```python
def evaluate(self, telemetry, now):
    reasons = tuple(sorted(self._reasons(telemetry, now)))
    return EligibilityResult(eligible=not reasons, reasons=reasons)
```

- [ ] **Step 5: Apply fail-closed transitions and Task Gate checks**

If a robot with a Block or unreleased authority becomes offline, broken, E-stopped, or position-unknown, use the existing Arbiter fault path. At a SafeStop, mark only the robot ineligible. Check eligibility in initial submission and each `_attempt_current_step`/`_attempt_chain_leg`.

```python
return {
    "decision": "BLOCKED",
    "reason": "robot_not_eligible",
    "eligibility_reasons": list(result.reasons),
}
```

Simulation profiles explicitly disable operational eligibility to preserve legacy tests; production validation forbids disabling it.

- [ ] **Step 6: Verify and commit**

Run: `.venv/bin/python -m unittest tests.test_robot_eligibility tests.test_fault_safety tests.test_block_occupancy tests.test_corridor_chain_task_gate tests.test_movement_authority -v`

Expected: PASS. Commit Task 3 files with message `Gate corridor entry on VDA5050 readiness`.

### Task 4: Health, Readiness, and Production Launcher

**Files:**
- Create: `traffic_control/readiness.py`
- Modify: `traffic_control/task_gate.py`
- Create: `tests/test_runtime_readiness.py`
- Create: `tests/test_task_gate_health.py`
- Create: `scripts/start_production_corridor.sh`
- Create: `scripts/stop_production_corridor.sh`
- Modify: `scripts/run_direction_arbiter.sh`
- Create: `tests/test_production_launcher.py`
- Modify: `README.md`
- Modify: `docs/simulation_run_guide.md`

**Interfaces:**
- Produces: `RuntimeReadiness.health() -> dict[str, Any]`.
- Produces: `RuntimeReadiness.ready(now=None) -> dict[str, Any]`.
- Produces: `RuntimeReadiness.can_accept_tasks(now=None) -> bool`.
- Produces: `/health`, `/ready`, and redacted readiness/eligibility fields in `/traffic/status`.

- [ ] **Step 1: Write failing readiness and API tests**

Assert pending before telemetry, ready only when all required robots occupy distinct SafeStops with empty Blocks, and sticky `recovery.required` after observing any robot inside a managed Block. `/health` stays 200 for a live process; `/ready` returns 503 when false and 200 when true.

```python
self.assertEqual(readiness.ready(now=1.0)["reason"], "telemetry.pending")
ingest_all_at_safe_stops(tracker, now=2.0)
self.assertTrue(readiness.ready(now=2.0)["ready"])
ingest_inside_block(tracker, "R1", now=3.0)
self.assertEqual(readiness.ready(now=3.0)["reason"], "recovery.required")
```

- [ ] **Step 2: Write failing production launcher tests**

Assert preflight appears before compose/Arbiter, external `RMF_API_BEARER_TOKEN` is required, and no simulator, visualizer, JWT creation, or broad `pkill` occurs.

- [ ] **Step 3: Run RED**

Run: `.venv/bin/python -m unittest tests.test_runtime_readiness tests.test_task_gate_health tests.test_production_launcher -v`

Expected: FAIL because readiness and production scripts do not exist.

- [ ] **Step 4: Implement readiness and wire Task Gate**

Cache the upstream probe for at most one second. Accept optional readiness in legacy Task Gate construction; production always supplies it. Do not cancel already-forwarded motion automatically.

- [ ] **Step 5: Implement validation-first start and scoped stop**

Start only configured RMF/Fleet Adapter/Arbiter services, never Simulator. Poll `/health` then `/ready` with a monotonic timeout. Stop only PID/container/service names from the deployment profile. Document field inputs and health versus readiness.

- [ ] **Step 6: Verify and commit**

Run: `.venv/bin/python -m unittest tests.test_runtime_readiness tests.test_task_gate_health tests.test_production_launcher tests.test_portable_workspace -v`

Run: `.venv/bin/python scripts/validate_production_deployment.py config/production.connected-corridor.example.yaml`

Expected: tests PASS; example preflight exits `2` with replacement/missing-secret codes. Commit Task 4 files with message `Add fail-closed production runtime`.

### Task 5: Generic Simulator Fault Injection

**Files:**
- Create: `rmf_dev_tool-main/vda5050_robot_simulator/vda5050_simulator/fault_injection.py`
- Modify: `rmf_dev_tool-main/vda5050_robot_simulator/run.py`
- Modify: `rmf_dev_tool-main/vda5050_robot_simulator/vda5050_simulator/state_publisher.py`
- Modify: `rmf_dev_tool-main/vda5050_robot_simulator/vda5050_simulator/mqtt_client.py`
- Create: `tests/test_simulator_fault_injection.py`
- Create: `rmf_dev_tool-main/vda5050_robot_simulator/connected_corridor_fault_scenarios.yaml`

**Interfaces:**
- Produces: `FaultInjectionController.from_config(items)`.
- Produces: `FaultInjectionController.apply(robot, elapsed) -> FaultEffects`.
- Trigger keys: `elapsed_at_least`, `at_node`, `driving`, combined with AND semantics.
- One-shot actions: `set_estop`, `set_operating_mode`, `set_paused`, `set_map_id`, `set_position_initialized`, `suppress_state`, `publish_connection`.

- [ ] **Step 1: Write failing controller tests**

Prove rules fire once, unrelated robots remain unaffected, robot/node IDs exist only in scenario YAML, invalid actions fail startup, and state suppression does not suppress connection publication.

```python
controller = FaultInjectionController.from_config([{
    "robot": "R1",
    "when": {"at_node": "N2", "driving": True},
    "action": {"type": "set_estop", "value": "AUTOACK"},
}])
effect = controller.apply(robot("R1", node="N2", driving=True), elapsed=3.0)
self.assertEqual(effect.state_overrides["safetyState"]["eStop"], "AUTOACK")
self.assertFalse(controller.apply(robot("R1", node="N2", driving=True), 4.0).triggered)
```

- [ ] **Step 2: Run RED**

Run: `.venv/bin/python -m unittest tests.test_simulator_fault_injection -v`

Expected: FAIL because the controller is missing.

- [ ] **Step 3: Implement deterministic triggers and ordinary VDA5050 effects**

Apply overrides immediately before serialization. `suppress_state` skips only state publication and never sleeps motion. Connection actions use the existing retained VDA5050 connection topic.

- [ ] **Step 4: Add disabled-by-default reusable examples**

Provide state loss inside the middle corridor, E-stop at a node, MANUAL at a SideStop, wrong map, and position-uninitialized examples. All robot/node values remain YAML fixtures.

- [ ] **Step 5: Verify and commit**

Run: `.venv/bin/python -m unittest tests.test_simulator_fault_injection tests.test_vda5050_edge_orientation -v`

Expected: PASS. Commit Task 5 files with message `Add configurable VDA5050 fault injection`.

### Task 6: Full Regression, Runtime Evidence, and Push

**Files:**
- Create: `scripts/run_connected_corridor_fault_scenario.py`
- Create: `tests/test_connected_corridor_fault_runner.py`
- Modify: `docs/simulation_run_guide.md`

**Interfaces:**
- Produces: condition-polling verifier with `--scenario`, `--timeout`, and JSON evidence.

- [ ] **Step 1: Write failing verifier tests**

Feed synthetic status/log snapshots and recognize: no authority while ineligible, Block fault for inside-corridor E-stop/state loss, no unnecessary Block fault for MANUAL at SafeStop, and no readiness recovery after an inside-corridor restart.

- [ ] **Step 2: Implement condition-based polling**

Use one overall monotonic deadline and state conditions, not a fixed verdict sleep. Record timestamps, robots, jobs, Blocks, readiness reasons and fault reasons as JSON.

- [ ] **Step 3: Run the full automated suite**

Run: `.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v`

Expected: more than 134 tests, zero failures and zero errors.

- [ ] **Step 4: Run normal dynamic 2v2**

Confirm all four jobs complete at configured endpoints, all Blocks are FREE, queues/reservations/authorities are empty, and no fault occurs.

- [ ] **Step 5: Run state-loss, E-stop, SafeStop-MANUAL, and restart scenarios**

Record a fresh log offset for each run. Confirm inside-corridor faults lock related authority and prevent opposite admission; SafeStop MANUAL blocks only that robot; clean start becomes ready only when all robots are safe.

- [ ] **Step 6: Re-run the complete suite and review hygiene**

Run: `.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v`

Run: `git diff --check && git status --short`

Expected: PASS; no `.runtime`, recording, DOCX, certificate, key, token, or environment-secret file staged.

- [ ] **Step 7: Commit runtime tooling and review all changes**

Commit Task 6 files with message `Validate fail-closed corridor fault scenarios`. Review `git diff --stat fe9e332..HEAD` and scan tracked content for private-key headers, literal passwords, and bearer JWTs.

- [ ] **Step 8: Push only after all checks pass**

Run: `git push origin feature/p4-single-passing-bay-poc`

Expected: the remote branch advances to the fully verified commit. Report remaining real-site inputs without claiming real-hardware validation.
