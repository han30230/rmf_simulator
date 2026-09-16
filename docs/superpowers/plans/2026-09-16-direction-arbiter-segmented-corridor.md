# Direction Arbiter Segmented Corridor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fail-safe, configuration-driven Direction Arbiter and RMF task admission proxy to the recovered VDA5050 simulation workspace without changing RMF Core or the robot simulator control path.

**Architecture:** A pure Python state machine owns directional domains, block reservations, and destination holding-bay reservations. A VDA5050 state tracker converts MQTT telemetry into enter/exit/fault events, while a task gate stages only admitted task segments to the existing RMF API at port 8100 and exposes its proxy on port 8200.

**Tech Stack:** Python 3.12, dataclasses, PyYAML, paho-mqtt, FastAPI, uvicorn, unittest

**Spec:** ChatGPT Library `/direction_arbiter_segmented_corridor_implementation_spec.md`

## Global Constraints

- Do not modify RMF Core Planner, Negotiation, or Schedule DB.
- Do not reject VDA5050 orders in `robot.py` or `order_manager.py`.
- Wait only at configured Holding/Parking/Refuge points; never stop an admitted robot inside a block.
- Reserve the next block and destination holding bay atomically.
- Use `agvPosition` as primary occupancy evidence; use `edgeStates`, `nodeStates`, and `lastNodeId` as supporting evidence.
- Telemetry loss while inside a block is fail-closed and sets the block to `BLOCKED`.
- Direction comes from explicit route configuration, never node-number or x-coordinate ordering.
- Current recovered two-node map is not treated as a safe segmented production map.

---

### Task 1: Core domain model and directional admission

**Files:**
- Create: `traffic_control/__init__.py`
- Create: `traffic_control/models.py`
- Create: `traffic_control/corridor_registry.py`
- Create: `traffic_control/direction_arbiter.py`
- Test: `tests/test_direction_arbiter.py`

**Interfaces:**
- Produces: `DirectionArbiter.request()`, `mark_entered()`, `mark_exited()`, `cancel()`, `fault()`, `reset()`, `snapshot()`.
- Produces: `CorridorRegistry.from_dict()` and `CorridorRegistry.from_yaml()`.

- [ ] Write tests for atomic block/HB admission, opposite-direction waiting, batch closure, direction switching after clear, independent directional domains, fault blocking, and reset safety.
- [ ] Run `python3 -m unittest tests.test_direction_arbiter -v` and confirm failure because the feature is absent.
- [ ] Implement the smallest thread-safe model and state machine satisfying the tests.
- [ ] Re-run the focused test and confirm all cases pass.

### Task 2: VDA5050 position and fault tracking

**Files:**
- Create: `traffic_control/robot_tracker.py`
- Test: `tests/test_block_occupancy.py`
- Test: `tests/test_fault_safety.py`

**Interfaces:**
- Consumes: registry geometry and DirectionArbiter lifecycle methods.
- Produces: `RobotTracker.ingest_state()`, `expire_stale()`, `current_safe_node()`, `snapshot()` and `MqttStateMonitor`.

- [ ] Write tests proving geometry-based ENTER/EXIT, node inference at an HB, no `lastNodeId`-only clear, and timeout fail-closed behavior.
- [ ] Run both focused test files and confirm expected failures.
- [ ] Implement state parsing, geometry classification, lifecycle transitions, and optional MQTT wildcard subscription.
- [ ] Re-run both focused test files and confirm all cases pass.

### Task 3: Segmented task gate and dynamic insertion

**Files:**
- Create: `traffic_control/task_gate.py`
- Test: `tests/test_dynamic_insert.py`

**Interfaces:**
- Consumes: `CorridorRegistry.resolve_route()`, tracker safe-node state, and arbiter decisions.
- Produces: `TaskGate.submit()`, `tick()`, `cancel()`, `reset()`, `status()` and FastAPI endpoints under port 8200.

- [ ] Write tests for manager-off passthrough, staged goals, opposite dynamic insertion, same-direction batch closure, cancellation, and upstream failure retention.
- [ ] Run the focused test and confirm expected failures.
- [ ] Implement a dependency-injected forwarding service, standard-library upstream HTTP client, and optional FastAPI wrapper.
- [ ] Re-run the focused test and confirm all cases pass.

### Task 4: Configuration, scripts, and operator documentation

**Files:**
- Create: `config/corridor_blocks.yaml`
- Create: `config/corridor_blocks_segmented_example.yaml`
- Create: `scripts/run_direction_arbiter.sh`
- Create: `scripts/t4_dispatch_via_arbiter.sh`
- Create: `docs/direction_arbiter_guide.md`
- Modify: `rmf_dev_tool-main/vda5050_robot_simulator/requirements.txt`
- Modify: `README.md`

**Interfaces:**
- The proxy listens on `127.0.0.1:8200` and forwards admitted stages to `http://127.0.0.1:8100/tasks/robot_task`.
- MQTT defaults to `127.0.0.1:1883` with `uagv/v2.0.0/inatech/+/state`.

- [ ] Add a disabled safe template for the recovered two-node map and an enabled three-block segmented example for an edited map.
- [ ] Add start and dispatch scripts that preserve the existing T1-T4 workflow.
- [ ] Document configuration, endpoints, logs, test scenarios, limitations, and migration to the Fleet Adapter.
- [ ] Validate both YAML files and shell syntax.

### Task 5: Full verification and distributable checkpoint

**Files:**
- Modify: `scripts/verify_workspace.py` only if needed to include the new checks.
- Create: workspace ZIP checkpoint after validation.

- [ ] Run `python3 -m unittest discover -s tests -v`.
- [ ] Run `python3 -m compileall traffic_control tests`.
- [ ] Run `python3 scripts/verify_workspace.py` and distinguish pre-existing missing external RMF assets from new failures.
- [ ] Run an in-process 1v1, 2v2, dynamic insertion, independent-block, pipeline, and fault simulation through the tests.
- [ ] Package the workspace without caches, logs, or virtual environments.
