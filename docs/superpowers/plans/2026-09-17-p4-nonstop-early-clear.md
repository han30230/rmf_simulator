# P4 Nonstop Early Clear Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow an admitted robot to continue through a route step without stopping while releasing only its shared corridor conflict resource at a configured telemetry node.

**Architecture:** Add an optional `release_node` to route steps and reservations. `RobotTracker` converts an exact `lastNodeId` match into a `CLEARED` state in `DirectionArbiter`; that state frees block and direction-domain resources but retains the destination holding-bay reservation until `TaskGate` confirms final arrival.

**Tech Stack:** Python 3.12, dataclasses, PyYAML, `unittest`, VDA5050 MQTT state telemetry, FastAPI task gate.

**Spec:** `docs/superpowers/specs/2026-09-17-p4-nonstop-early-clear-design.md`

## Global Constraints

- The upstream A1 task remains one `2104 -> 2108` task; no task may be inserted at `2106`.
- Early release requires an exact configured `lastNodeId`; position, elapsed time, and ETA alone must never release the conflict resource.
- Destination holding-bay reservation must remain held until actual destination arrival.
- Existing routes without `release_node` must retain current behavior.
- Production Python must contain no P4 robot IDs, node IDs, or block IDs.
- Missing release telemetry must fail closed and preserve the existing timeout behavior.
- The implementation covers the P4 1-v-1 scenario only; 2-v-1, 2-v-2, and multi-corridor policy remain follow-up work.

---

### Task 1: Parse and validate `release_node`

**Files:**
- Modify: `traffic_control/models.py`
- Modify: `traffic_control/corridor_registry.py`
- Modify: `tests/test_direction_arbiter.py`

**Interfaces:**
- Produces: `RouteStep.release_node: str | None`
- Produces: `Reservation.release_node: str | None`
- Validation rule: release node differs from `goal_node` and occurs in the selected direction's edge list.

- [ ] **Step 1: Write failing registry tests**

Add a small helper and three tests to `tests/test_direction_arbiter.py`:

```python
def release_route_config(release_node: str) -> dict:
    return {
        "holding_bays": {
            "HB0": {"node_id": "N0"},
            "HB2": {"node_id": "N2"},
        },
        "blocks": [
            {
                "id": "TOP",
                "entry_a": "HB0",
                "entry_b": "HB2",
                "edges_a_to_b": ["N0>N1", "N1>N2"],
                "edges_b_to_a": ["N2>N1", "N1>N0"],
            }
        ],
        "routes": [
            {
                "start_nodes": ["N0"],
                "goal_nodes": ["N2"],
                "steps": [
                    {
                        "block_id": "TOP",
                        "direction": "A_TO_B",
                        "source_hb": "HB0",
                        "destination_hb": "HB2",
                        "goal_node": "N2",
                        "release_node": release_node,
                    }
                ],
            }
        ],
    }


def test_release_node_is_parsed(self) -> None:
    registry = CorridorRegistry.from_dict(release_route_config("N1"))
    self.assertEqual(registry.routes[0].steps[0].release_node, "N1")


def test_release_node_must_belong_to_direction_edges(self) -> None:
    with self.assertRaisesRegex(ValueError, "release node"):
        CorridorRegistry.from_dict(release_route_config("OTHER"))


def test_release_node_must_precede_goal(self) -> None:
    with self.assertRaisesRegex(ValueError, "release node"):
        CorridorRegistry.from_dict(release_route_config("N2"))
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run:

```bash
python -m unittest tests.test_direction_arbiter.DirectionArbiterTests.test_release_node_is_parsed tests.test_direction_arbiter.DirectionArbiterTests.test_release_node_must_belong_to_direction_edges tests.test_direction_arbiter.DirectionArbiterTests.test_release_node_must_precede_goal -v
```

Expected: the parse assertion fails because `RouteStep` has no `release_node`; validation tests also fail because invalid values are accepted.

- [ ] **Step 3: Add the optional model fields**

Update the dataclasses in `traffic_control/models.py`:

```python
@dataclass(frozen=True)
class Reservation:
    robot_id: str
    block_id: str
    direction: Direction
    destination_hb: str
    source_hb: str | None
    request_time: float
    release_node: str | None = None


@dataclass(frozen=True)
class RouteStep:
    block_id: str
    direction: Direction
    destination_hb: str
    goal_node: str
    source_hb: str | None = None
    release_node: str | None = None
```

- [ ] **Step 4: Implement registry normalization and validation**

In `CorridorRegistry.from_dict`, normalize `step.get("release_node")`, derive the node set from `block.edges_a_to_b` or `block.edges_b_to_a`, and reject invalid values before constructing `RouteStep`:

```python
release_node = step.get("release_node")
normalized_release = None if release_node is None else str(release_node).strip()
if normalized_release == "":
    raise ValueError(f"route step {block_id} release node must not be empty")
if normalized_release is not None:
    direction_edges = (
        block.edges_a_to_b
        if direction is Direction.A_TO_B
        else block.edges_b_to_a
    )
    direction_nodes = {
        node
        for edge in direction_edges
        for node in edge.split(">", maxsplit=1)
    }
    if normalized_release == goal_node or normalized_release not in direction_nodes:
        raise ValueError(
            f"route step {block_id} release node {normalized_release} "
            f"must precede goal {goal_node} on {direction.value} edges"
        )
```

Pass `release_node=normalized_release` into `RouteStep`.

- [ ] **Step 5: Run focused and existing registry tests**

Run:

```bash
python -m unittest tests.test_direction_arbiter tests.test_config_files -v
```

Expected: all tests pass.

- [ ] **Step 6: Commit the model and registry unit**

```bash
git add traffic_control/models.py traffic_control/corridor_registry.py tests/test_direction_arbiter.py
git commit -m "Add configurable corridor release nodes"
```

---

### Task 2: Separate conflict clearance from final arrival

**Files:**
- Modify: `traffic_control/models.py`
- Modify: `traffic_control/direction_arbiter.py`
- Modify: `tests/test_direction_arbiter.py`

**Interfaces:**
- Consumes: `Reservation.release_node`
- Produces: `RobotCorridorState.CLEARED`
- Produces: `DirectionArbiter.mark_cleared(robot_id: str, block_id: str) -> bool`
- Produces: `DirectionArbiter.mark_arrived(robot_id: str, block_id: str) -> bool`
- Produces: `DirectionArbiter.release_node_for_robot(robot_id: str) -> tuple[str, str] | None`

- [ ] **Step 1: Write a failing resource-lifetime test**

Add this test to `DirectionArbiterTests`:

```python
def test_early_clear_releases_domain_but_retains_destination_bay(self) -> None:
    arbiter = DirectionArbiter(make_registry())
    self.assertIs(
        arbiter.request(
            "A1", "TOP_1", Direction.A_TO_B, "HB1",
            source_hb="HB0", release_node="N_CLEAR",
        ),
        Decision.ADMIT,
    )
    self.assertIs(
        arbiter.request("B1", "TOP_2", Direction.B_TO_A, "HB1"),
        Decision.WAIT,
    )
    arbiter.mark_entered("A1", "TOP_1")

    self.assertTrue(arbiter.mark_cleared("A1", "TOP_1"))

    status = arbiter.snapshot()
    self.assertEqual(status["blocks"]["TOP_1"]["occupants"], [])
    self.assertEqual(status["holding_bays"]["HB1"]["reservations"], ["A1"])
    self.assertIsNone(arbiter.active_block_for_robot("A1"))
    self.assertIs(arbiter.decision_for("B1", "TOP_2"), Decision.ADMIT)

    self.assertTrue(arbiter.mark_arrived("A1", "TOP_1"))
    self.assertFalse(arbiter.mark_arrived("A1", "TOP_1"))
    status = arbiter.snapshot()
    self.assertEqual(status["holding_bays"]["HB1"]["reservations"], [])
    self.assertIn("A1", status["holding_bays"]["HB1"]["occupants"])
```

- [ ] **Step 2: Run the test and confirm RED**

Run:

```bash
python -m unittest tests.test_direction_arbiter.DirectionArbiterTests.test_early_clear_releases_domain_but_retains_destination_bay -v
```

Expected: `request()` rejects `release_node` or `mark_cleared` is missing.

- [ ] **Step 3: Carry release metadata into grants**

Extend `DirectionArbiter.request`:

```python
def request(
    self,
    robot_id: str,
    block_id: str,
    direction: Direction | str,
    destination_hb: str,
    *,
    source_hb: str | None = None,
    request_time: float | None = None,
    release_node: str | None = None,
) -> Decision:
```

Set `release_node=release_node` when creating `Reservation`.

- [ ] **Step 4: Implement the two-phase lifecycle**

Add `CLEARED = "CLEARED"` to `RobotCorridorState`. Implement these methods under the Arbiter lock:

```python
def mark_cleared(self, robot_id: str, block_id: str) -> bool:
    with self._lock:
        key = (robot_id, block_id)
        reservation = self._grants.get(key)
        if reservation is None or reservation.release_node is None:
            return False
        if self._robot_states.get(key) is RobotCorridorState.CLEARED:
            return False
        block = self.registry.blocks[block_id]
        block.occupants.pop(robot_id, None)
        block.reservations.pop(robot_id, None)
        self._robot_states[key] = RobotCorridorState.CLEARED
        logger.info(
            "[TRAFFIC] ROBOT_CLEARED_BLOCK robot=%s block=%s release_node=%s",
            robot_id, block_id, reservation.release_node,
        )
        self._reconcile_domain(block.direction_domain)
        self._assert_invariants()
        return True


def mark_arrived(self, robot_id: str, block_id: str) -> bool:
    with self._lock:
        key = (robot_id, block_id)
        reservation = self._grants.pop(key, None)
        if reservation is None:
            return False
        block = self.registry.blocks[block_id]
        block.occupants.pop(robot_id, None)
        block.reservations.pop(robot_id, None)
        destination = self.registry.holding_bays[reservation.destination_hb]
        destination.reservations.discard(robot_id)
        destination.occupants.add(robot_id)
        self._robot_states[key] = RobotCorridorState.EXITED
        self._reconcile_domain(block.direction_domain)
        self._assert_invariants()
        return True
```

Add `release_node_for_robot` and filter cleared grants from `active_block_for_robot`:

```python
def release_node_for_robot(self, robot_id: str) -> tuple[str, str] | None:
    with self._lock:
        matches = [
            (block_id, reservation.release_node)
            for (candidate, block_id), reservation in self._grants.items()
            if candidate == robot_id
            and reservation.release_node is not None
        ]
        return matches[0] if len(matches) == 1 else None
```

At the start of `mark_entered`, return without changing resources when the key is already `RobotCorridorState.CLEARED`. This is a second idempotency guard against repeated release-node telemetry.

- [ ] **Step 5: Run Arbiter tests**

Run:

```bash
python -m unittest tests.test_direction_arbiter -v
```

Expected: all tests pass, including existing normal `mark_exited` behavior.

- [ ] **Step 6: Commit the Arbiter lifecycle**

```bash
git add traffic_control/models.py traffic_control/direction_arbiter.py tests/test_direction_arbiter.py
git commit -m "Release corridor conflicts before final arrival"
```

---

### Task 3: Trigger clearance from VDA5050 telemetry and finalize in TaskGate

**Files:**
- Modify: `traffic_control/robot_tracker.py`
- Modify: `traffic_control/task_gate.py`
- Modify: `tests/test_block_occupancy.py`

**Interfaces:**
- Consumes: `DirectionArbiter.release_node_for_robot()`
- Consumes: `DirectionArbiter.mark_cleared()` and `mark_arrived()`
- Behavior: exact release-node telemetry clears `current_block`; final destination arrival completes retained grant.

- [ ] **Step 1: Write a failing tracker test**

Extend `make_components` with `release_node: str | None = None`, pass directional edges as node-pair strings, and pass `release_node` to the Arbiter request. Add:

```python
def test_exact_release_node_clears_block_while_robot_keeps_driving(self) -> None:
    arbiter, tracker = make_components()
    arbiter.request(
        "A1", "TOP_1", Direction.A_TO_B, "HB1",
        source_hb="HB0", release_node="N_CLEAR",
    )
    tracker.ingest_state("A1", state(5.0, last_node="N_BEFORE"), received_at=1.0)
    self.assertEqual(tracker.snapshot()["A1"]["current_block"], "TOP_1")

    tracker.ingest_state(
        "A1", state(9.0, last_node="N_CLEAR", driving=True), received_at=2.0
    )

    snapshot = tracker.snapshot()["A1"]
    self.assertIsNone(snapshot["current_block"])
    self.assertTrue(snapshot["driving"])
    status = arbiter.snapshot()
    self.assertEqual(status["blocks"]["TOP_1"]["occupants"], [])
    self.assertEqual(status["holding_bays"]["HB1"]["reservations"], ["A1"])

    tracker.ingest_state(
        "A1", state(9.0, last_node="N_CLEAR", driving=True), received_at=2.5
    )
    self.assertIsNone(tracker.snapshot()["A1"]["current_block"])
    self.assertEqual(arbiter.snapshot()["blocks"]["TOP_1"]["occupants"], [])
```

Also add a test that `lastNodeId="N_BEFORE"` and an outside position do not call early clear; after timeout, the block must be faulted.

- [ ] **Step 2: Run the tracker tests and confirm RED**

Run:

```bash
python -m unittest tests.test_block_occupancy.BlockOccupancyTests.test_exact_release_node_clears_block_while_robot_keeps_driving -v
```

Expected: `current_block` remains `TOP_1` because tracker does not inspect release metadata.

- [ ] **Step 3: Implement exact node-triggered clearance**

In `RobotTracker.ingest_state`, normalize `last_node_id` once near the top and calculate an exact release match before position/edge transitions:

```python
last_node_id = str(payload.get("lastNodeId") or "")
release = self.arbiter.release_node_for_robot(robot_id)
release_matches = release is not None and last_node_id == release[1]
```

When `release_matches` is true, bypass block entry detection for that message. After the normal geometry/edge branches, apply the idempotent clear and force the tracker outside the conflict block:

```python
if release_matches:
    release_block, _ = release
    self.arbiter.mark_cleared(robot_id, release_block)
    block_id = None
    hb_id = None
```

Use `last_node_id` when constructing `RobotTelemetry`. Do not infer a holding bay at the release node.

- [ ] **Step 4: Finalize retained grants only at final arrival**

In `TaskGate.tick`, immediately after `has_arrived` succeeds and before incrementing `step_index`, add:

```python
self.arbiter.mark_arrived(job.robot_id, step.block_id)
```

In `_attempt_current_step`, pass `release_node=step.release_node` to `arbiter.request`.

- [ ] **Step 5: Run tracker, gate, and fault regression tests**

Run:

```bash
python -m unittest tests.test_block_occupancy tests.test_dynamic_insert tests.test_fault_safety -v
```

Expected: all tests pass; the new timeout test shows fail-closed behavior before release.

- [ ] **Step 6: Commit telemetry integration**

```bash
git add traffic_control/robot_tracker.py traffic_control/task_gate.py tests/test_block_occupancy.py
git commit -m "Clear corridor grants at telemetry release nodes"
```

---

### Task 4: Configure and verify the P4 nonstop handoff

**Files:**
- Modify: `config/corridor_blocks_p4_passing_bay.yaml`
- Modify: `tests/test_passing_bay_config.py`
- Modify: `tests/test_passing_bay_flow.py`

**Interfaces:**
- Consumes: route-step `release_node`
- P4 release boundary: node `2106`
- P4 A1/east-side central conflict geometry: `31.1 <= x <= 54.818`
- P4 side-to-left route geometry: `7.6 <= x <= 54.818`

- [ ] **Step 1: Write failing P4 configuration assertions**

In `tests/test_passing_bay_config.py`, assert:

```python
forward = next(
    route for route in registry.routes
    if route.route_id == "P4_LEFT_TO_RIGHT_VIA_GATE"
)
self.assertEqual(forward.steps[1].release_node, "2106")

for block_id in ("P4_EAST_TO_SIDE", "P4_GATE_TO_RIGHT"):
    bounds = registry.blocks[block_id].geometry["bounds"]
    self.assertEqual(float(bounds["min_x"]), 31.1)
    self.assertEqual(float(bounds["max_x"]), 54.818)

side_to_left = registry.blocks["P4_SIDE_TO_LEFT"].geometry["bounds"]
self.assertEqual(float(side_to_left["min_x"]), 7.6)
self.assertEqual(float(side_to_left["max_x"]), 54.818)
```

- [ ] **Step 2: Rewrite the passing-bay flow expectation before changing config**

Add `2105`, `2106`, and `2107` to `POSITIONS`, then change `test_b1_yields_in_side_bay_then_rejoins_after_a1_crosses` so it proves:

```python
tracker.ingest_state(
    "AGV_A1", state(*POSITIONS["2105"], node="2105", driving=True),
    received_at=5.0,
)
gate.tick(now=5.0)
self.assertEqual(forwarder.goals[-1], ("AGV_A1", "2108"))
self.assertEqual(gate.status()["jobs"][b1["job_id"]]["status"], "WAITING")

tracker.ingest_state(
    "AGV_A1", state(*POSITIONS["2106"], node="2106", driving=True),
    received_at=5.5,
)
gate.tick(now=5.5)
self.assertEqual(forwarder.goals[-1], ("AGV_B1", "2101"))

handoff = gate.status()
self.assertEqual(handoff["jobs"][a1["job_id"]]["status"], "ACTIVE")
self.assertTrue(handoff["robots"]["AGV_A1"]["driving"])
self.assertIsNone(handoff["robots"]["AGV_A1"]["current_block"])
self.assertIn(
    "AGV_A1",
    handoff["arbiter"]["holding_bays"]["HB_RIGHT"]["reservations"],
)
```

Continue the test with A1 at `2107` while B1 enters `P4_SIDE_TO_LEFT`, then deliver A1 to `2108` and B1 to `2101`. Assert both jobs complete and every block has no fault, occupants, reservations, or waiters.

- [ ] **Step 3: Run P4 tests and confirm RED**

Run:

```bash
python -m unittest tests.test_passing_bay_config tests.test_passing_bay_flow -v
```

Expected: release-node assertion fails and B1 is not dispatched at `2106`.

- [ ] **Step 4: Update P4 configuration**

Preserve the previously validated west-approach gap fix:

```yaml
bounds: {min_x: 7.6, max_x: 29.9, min_y: 91.8, max_y: 93.9}
```

Replace the shared geometry anchor with:

```yaml
bounds: &passing_main_bounds {min_x: 31.1, max_x: 54.818, min_y: 91.75, max_y: 93.9}
```

Override `P4_SIDE_TO_LEFT` because that granted movement continues through the west approach:

```yaml
bounds: {min_x: 7.6, max_x: 54.818, min_y: 91.75, max_y: 93.9}
```

Add to the second `P4_LEFT_TO_RIGHT_VIA_GATE` step:

```yaml
release_node: "2106"
```

- [ ] **Step 5: Run P4 tests and confirm GREEN**

Run:

```bash
python -m unittest tests.test_passing_bay_config tests.test_passing_bay_flow -v
```

Expected: all tests pass and the flow test proves B1 dispatch before A1 reaches `2108`.

- [ ] **Step 6: Commit P4 behavior**

```bash
git add config/corridor_blocks_p4_passing_bay.yaml tests/test_passing_bay_config.py tests/test_passing_bay_flow.py
git commit -m "Release P4 passing conflict at node 2106"
```

---

### Task 5: Full verification and runtime handoff

**Files:**
- Verify: all `traffic_control/`, `config/`, and `tests/` changes
- Update only if commands or expected logs changed: `P4_HOME_PATCH_README.md`

**Interfaces:**
- Produces: a branch commit set ready for the user's Ubuntu-24.04 runtime test.

- [ ] **Step 1: Run the full unit suite**

Run:

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: zero failures and zero errors.

- [ ] **Step 2: Check syntax and configuration loading**

Run:

```bash
python -m compileall -q traffic_control tests
python - <<'PY'
from traffic_control.corridor_registry import CorridorRegistry

r = CorridorRegistry.from_yaml("config/corridor_blocks_p4_passing_bay.yaml")
route = next(x for x in r.routes if x.route_id == "P4_LEFT_TO_RIGHT_VIA_GATE")
assert route.steps[1].goal_node == "2108"
assert route.steps[1].release_node == "2106"
print("P4 nonstop early-clear config: PASS")
PY
```

Expected: compile exits 0 and the configuration script prints `PASS`.

- [ ] **Step 3: Inspect the final diff for hardcoding and unintended files**

Run:

```bash
git diff origin/feature/p4-single-passing-bay-poc...HEAD --stat
git diff origin/feature/p4-single-passing-bay-poc...HEAD -- traffic_control
grep -RInE 'AGV_A1|P4_GATE_TO_RIGHT|2106' traffic_control || true
```

Expected: P4 identifiers appear only in configuration/tests/docs, not production `traffic_control` code.

- [ ] **Step 4: Push and verify the remote branch**

```bash
git push origin feature/p4-single-passing-bay-poc
git fetch origin
test "$(git rev-parse HEAD)" = "$(git rev-parse origin/feature/p4-single-passing-bay-poc)"
```

Expected: local and remote branch SHAs match.

- [ ] **Step 5: Give the runtime validation sequence**

The user will pull after preserving the local `max_x: 29.9` edit, rebuild `vda5050_fleet_adapter`, restart the two-robot simulator and Arbiter, dispatch the passing-bay scenario, and verify this event ordering:

```text
TASK_RELEASED AGV_A1 goal=2108
ROBOT_CLEARED_BLOCK AGV_A1 release_node=2106
TASK_RELEASED AGV_B1 goal=2101
TASK_COMPLETED AGV_A1
TASK_COMPLETED AGV_B1
```

The GUI must show A1 continuing through `2106` without stopping while B1 departs `6137` after A1 clears that node.
