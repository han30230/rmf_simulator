# Connected Corridor Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a configuration-driven connected three-segment single-lane corridor with two side bays, rolling multi-block movement authorities, and validated four-robot 1v3 and 2v2 VDA5050 simulations.

**Architecture:** Preserve the current single-block route path and add an opt-in corridor-chain path. A pure chain planner maps physical endpoint slots into ordered safe-stop groups and selects the farthest conflict-free refuge; `DirectionArbiter` atomically owns the resulting ordered block interval and one destination slot, while `TaskGate` submits one RMF task for the entire authority and `RobotTracker` releases passed blocks from telemetry.

**Tech Stack:** Python 3.12, `unittest`, PyYAML, Bash, Docker Compose, Open-RMF, MQTT, VDA5050 Simulator, PyQt5 Visualizer.

**Spec:** `docs/superpowers/specs/2026-09-19-connected-corridor-chain-design.md`

## Global Constraints

- Work only in `/home/han30230/rmf-work/rmf_passing_bay_poc` on `feature/p4-single-passing-bay-poc`; do not create a worktree or inspect another checkout.
- Preserve `recording_260919.jsonl` and all unrelated local changes; never stage `.runtime` or recording files.
- Do not branch on robot IDs, scenario cardinality, or connected-map node IDs in production Python.
- Keep the existing route/Reservation API working for the validated single passing-bay, staging, and independent multi-corridor scenarios.
- Make topology, physical slots, capacities, geometries, release nodes, chain order, and maximum active robots configuration data.
- A block used by a configured chain may name logical safe-stop groups as its `entry_a`/`entry_b`; existing static-route blocks continue to name physical holding bays.
- A normal planned stop may occur only in a configured physical safe-stop slot; junctions are not safe stops.
- Reserve every block in an authority plus its concrete destination slot atomically; never leave partial reservations.
- Do not revoke a movement authority already submitted to RMF.
- Do not widen geometry or add timing sleeps to hide a state transition defect.
- Use RED-GREEN TDD and retain the failing-test evidence in the work log.
- Do not start actual simulation until focused and full tests pass.
- Commit and push implementation only after connected 1v3 and connected 2v2 runtime acceptance both pass.

---

### Task 1: Parse physical slots, safe-stop groups, and ordered chains

**Files:**
- Modify: `traffic_control/models.py`
- Modify: `traffic_control/corridor_registry.py`
- Create: `tests/test_corridor_chain_registry.py`

**Interfaces:**
- Produces: `SafeStopGroup`, `CorridorChain`, `ChainPath`; `CorridorRegistry.safe_stop_groups`, `CorridorRegistry.corridor_chains`, `CorridorRegistry.resolve_chain_path(start_node, goal_node)`.
- Preserves: `resolve_route(s)` and every current YAML file that has no `corridor_chains` section.

- [x] **Step 1: Write registry RED tests**

Use a three-block fixture whose left and right groups each contain two physical
slots. Assert forward/reverse paths and strict validation:

```python
path = registry.resolve_chain_path("L2", "R1")
self.assertEqual(path.chain_id, "MAIN")
self.assertEqual(path.direction, Direction.A_TO_B)
self.assertEqual(path.block_ids, ("C1", "C2", "C3"))
self.assertEqual(path.source_group, "LEFT")
self.assertEqual(path.destination_group, "RIGHT")
self.assertEqual(path.source_slot, "LEFT_2")
self.assertEqual(path.destination_slot, "RIGHT_1")
```

Also assert rejection when `len(safe_stops) != len(blocks) + 1`, a group member
is unknown or duplicated, a block endpoint does not match its adjacent groups,
or `max_active_robots < 1`.

- [x] **Step 2: Run RED**

Run:

```bash
.venv/bin/python -m unittest tests.test_corridor_chain_registry -v
```

Expected: import or attribute failure because chain types and parsing do not exist.

- [x] **Step 3: Add immutable topology types**

Add these public shapes to `models.py`:

```python
@dataclass(frozen=True)
class SafeStopGroup:
    group_id: str
    members: tuple[str, ...]

@dataclass(frozen=True)
class CorridorChain:
    chain_id: str
    block_ids: tuple[str, ...]
    safe_stop_ids: tuple[str, ...]
    max_active_robots: int

@dataclass(frozen=True)
class ChainPath:
    chain_id: str
    direction: Direction
    block_ids: tuple[str, ...]
    safe_stop_ids: tuple[str, ...]
    source_group: str
    destination_group: str
    source_slot: str
    destination_slot: str
```

- [x] **Step 4: Parse and resolve chains**

Build reverse indexes from holding-bay node to physical slot and from slot to
safe-stop group. Permit `CorridorBlock.entry_a/entry_b` to resolve to either a
physical holding bay or a logical safe-stop group, then require every chain
block to match its adjacent group IDs in A-to-B order. Static `RouteStep`
validation still requires physical holding-bay endpoints. `resolve_chain_path`
returns `None` when either node is outside configured chains. Slice `block_ids`
in travel order and reverse them for `B_TO_A`; do not enumerate robot identities
or start/goal pairs.

- [x] **Step 5: Run GREEN plus registry regressions**

```bash
.venv/bin/python -m unittest \
  tests.test_corridor_chain_registry \
  tests.test_direction_arbiter \
  tests.test_passing_bay_config -v
```

Expected: all pass.

---

### Task 2: Select the farthest safe conflict-free leg

**Files:**
- Create: `traffic_control/corridor_chain.py`
- Create: `tests/test_corridor_chain_planner.py`

**Interfaces:**
- Consumes: `CorridorRegistry`, `ChainPath`, an availability callback with signature `Callable[[tuple[str, ...], Direction, str], bool]`.
- Produces: `PlannedAuthority` and `CorridorChainPlanner.plan(path, available) -> PlannedAuthority | None`.

- [x] **Step 1: Write planner RED tests**

Prove that a clear chain chooses the final physical slot, a conflict on C3
selects `SIDE_2`, a conflict on C2 selects `SIDE_1`, and no reachable refuge
returns `None`:

```python
leg = planner.plan(path, available)
self.assertEqual(leg.block_ids, ("C1", "C2"))
self.assertEqual(leg.destination_group, "SIDE_2_STOP")
self.assertEqual(leg.destination_slot, "SIDE_2")
self.assertEqual(leg.goal_node, "S2")
```

- [x] **Step 2: Run RED**

Expected: module-not-found failure.

- [x] **Step 3: Implement the pure planner**

Define:

```python
@dataclass(frozen=True)
class PlannedAuthority:
    chain_id: str
    direction: Direction
    source_group: str
    source_slot: str
    destination_group: str
    destination_slot: str
    goal_node: str
    block_ids: tuple[str, ...]
```

Evaluate candidate safe stops from final destination back toward the source.
For endpoint groups, retain the requested concrete slot. For intermediate
groups, choose the first free configured member in stable YAML order. Call the
availability callback with the full ordered interval and candidate slot.

- [x] **Step 4: Run GREEN**

Run the planner and registry modules; expected: all pass.

---

### Task 3: Add atomic multi-block movement authorities

**Files:**
- Modify: `traffic_control/models.py`
- Modify: `traffic_control/direction_arbiter.py`
- Create: `tests/test_movement_authority.py`

**Interfaces:**
- Produces: `MovementAuthority`; `DirectionArbiter.can_reserve_path`, `request_authority`, `authority_for_robot`, `mark_authority_arrived`, and `cancel_authority`.
- Preserves: existing `request`, `mark_arrived`, `cancel`, snapshots, and single-block semantics.

- [x] **Step 1: Write atomicity and overlap RED tests**

Assert that a three-block authority reserves all blocks and one destination
slot, a conflict on the final block leaves C1/C2 and the bay untouched, same
direction capacity can pipeline, opposite authorities may use disjoint C1/C3,
and no opposite authority may overlap C2.

```python
decision = arbiter.request_authority(planned, robot_id="A1", request_time=1.0)
self.assertEqual(decision, Decision.ADMIT)
self.assertEqual(
    arbiter.authority_for_robot("A1").unreleased_blocks,
    ("C1", "C2", "C3"),
)
```

- [x] **Step 2: Run RED**

Expected: missing authority APIs.

- [x] **Step 3: Add the authority model and atomic admission**

Use:

```python
@dataclass
class MovementAuthority:
    authority_id: str
    robot_id: str
    chain_id: str
    direction: Direction
    source_group: str
    source_slot: str
    destination_group: str
    destination_slot: str
    goal_node: str
    block_ids: tuple[str, ...]
    request_time: float
    released_blocks: set[str] = field(default_factory=set)

    @property
    def unreleased_blocks(self) -> tuple[str, ...]:
        return tuple(item for item in self.block_ids if item not in self.released_blocks)
```

Inside the Arbiter lock, validate the chain admission limit, every block fault,
capacity and direction, every involved direction-domain batch, and destination
slot capacity before mutating any resource. Then add one compatible
`Reservation` per block and reserve the destination slot once.

- [x] **Step 4: Add deterministic waiting and cancellation**

Keep pending authorities ordered by `(request_time, authority_id)`. An opposite
pending interval closes only the overlapping active domains. Cancellation
removes every pending/granted block reservation and the one destination
reservation. A rejected atomic request must leave snapshots byte-for-byte
equivalent except for its waiting entry.

- [x] **Step 5: Run GREEN and existing Arbiter tests**

Run movement-authority, direction-arbiter, fault-safety, and block-occupancy
modules; expected: all pass.

---

### Task 4: Track and release an authority across consecutive blocks

**Files:**
- Modify: `traffic_control/robot_tracker.py`
- Modify: `traffic_control/direction_arbiter.py`
- Modify: `tests/test_movement_authority.py`
- Modify: `tests/test_block_occupancy.py`

**Interfaces:**
- Consumes: active `MovementAuthority` and existing configured edge/geometry/release-node telemetry.
- Produces: ordered block entry/clear transitions and fail-closed timeout of all unreleased blocks.

- [x] **Step 1: Write telemetry RED tests**

Feed positions/edges from C1 through C3. Assert that the tracker selects the
matching block from the robot's authority, releases C1 then C2 in order, keeps
the destination slot reserved, and completes only at the concrete destination
node. Assert timeout faults every unreleased block rather than only the current
one.

- [x] **Step 2: Run RED**

Expected: the current single-grant lookup is ambiguous and blocks are not
released as an ordered authority.

- [x] **Step 3: Generalize granted-block classification**

Add `granted_blocks_for_robot(robot_id) -> tuple[str, ...]`. Robot Tracker first
chooses a geometry/edge match within that ordered set, then applies existing
source/destination safe-area rules, then reports unrelated block intrusion.
Never classify an unvisited future block as occupied solely because it is
reserved.

- [x] **Step 4: Release and fault authority resources**

`mark_cleared` records a released block but retains later reservations and the
destination slot. `fault(robot_id)` marks every block in
`authority.unreleased_blocks` faulted and logs each block. Arrival clears all
remaining reservations, moves physical slot occupancy, and deletes the active
authority.

- [x] **Step 5: Run GREEN and occupancy regressions**

Run the two modified modules plus existing passing-bay flow tests; expected:
all pass.

---

### Task 5: Integrate rolling authorities into Task Gate

**Files:**
- Modify: `traffic_control/task_gate.py`
- Create: `tests/test_corridor_chain_task_gate.py`

**Interfaces:**
- Consumes: `resolve_chain_path`, `CorridorChainPlanner`, authority Arbiter APIs.
- Produces: chain-aware `GateJob` progress while retaining current configured `RouteIntent` behavior.

- [ ] **Step 1: Write clear-chain and conflict RED tests**

For a chain task, assert one forwarded RMF goal at the final slot when clear.
With C3 conflicted, assert the first forwarded goal is SIDE_2, the job remains
active toward the original final goal, and the final leg is submitted only
after confirmed side-bay arrival. Assert an explicit HTTP 401 cancels the whole
authority while an uncertain response retains it.

- [ ] **Step 2: Run RED**

Expected: `TaskGate.submit` reports `unmatched_managed_route` because no static
route exists.

- [ ] **Step 3: Add opt-in chain jobs**

Extend `GateJob` with optional fields:

```python
final_goal_node: str | None = None
chain_path: ChainPath | None = None
active_authority_id: str | None = None
```

Try existing configured routes first. If none matches, call
`resolve_chain_path`; only chain-matched requests use the new planner. Rewrite
the upstream task goal to `PlannedAuthority.goal_node`. After authority arrival,
finish if it equals `final_goal_node`; otherwise resolve and plan from the new
physical slot.

- [ ] **Step 4: Preserve delivery failure semantics**

Explicit 4xx and explicit upstream rejection cancel every authority resource.
An exception or ambiguous upstream response marks the leg active and retains
the authority. Status exposes chain ID, final goal, active authority ID, source
slot, destination slot, and ordered blocks without changing existing job keys.

- [ ] **Step 5: Run GREEN and Task Gate regressions**

Run chain Task Gate, dynamic insert, passing-bay flow, and staging flow tests;
expected: all pass.

---

### Task 6: Add the connected three-corridor simulation stack

**Files:**
- Create: `config/corridor_blocks_connected_chain.yaml`
- Create: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/connected_corridor_chain.yaml`
- Create: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/config/connected_corridor_chain.yaml`
- Create: `rmf_dev_tool-main/vda5050_robot_simulator/connected_corridor_chain_scenario.yaml`
- Create: `rmf_platform-main/docker-compose.connected-corridor-chain.yml`
- Create: `scripts/start_connected_corridor_chain.sh`
- Create: `scripts/launch_connected_corridor_chain_visualizer.sh`
- Create: `scripts/dispatch_connected_corridor_chain_1v3.sh`
- Create: `scripts/dispatch_connected_corridor_chain_2v2.sh`
- Create: `tests/test_connected_corridor_chain_config.py`

**Interfaces:**
- Consumes: generic runtime selectors in `start_p4_passing_bay.sh` and the new chain schema.
- Produces: one connected main graph, C1/C2/C3, two physical side bays, distinct endpoint slots, and four registered robots.

- [ ] **Step 1: Write missing-file and semantic RED tests**

Assert that the map is one connected component; each side bay has one spur to a
distinct main junction; C1/C2/C3 form one ordered chain; junctions are absent
from all safe-stop members; endpoint slots are distinct; every configured edge
exists in the graph; no robot ID appears in Arbiter YAML; and all scripts are
repository-relative.

- [ ] **Step 2: Run RED**

Expected: new runtime files do not exist.

- [ ] **Step 3: Create map, configuration, and registrations**

Use symbolic map names such as `CHAIN_LEFT_1`, `CHAIN_SIDE_1`, and
`CHAIN_RIGHT_1` only in data files. Give C1/C2/C3 separate direction domains
and configured release nodes beyond each physical conflict boundary. Place
four simulator robots in distinct endpoint slots.

- [ ] **Step 4: Add launch and dispatch wrappers**

The start wrapper exports selected scenario, Compose, Arbiter config, and
runtime name before delegating to the existing launcher. Dispatch scripts only
map scenario robots to physical destination slot names; they contain no control
policy or sleep-based admission decisions.

- [ ] **Step 5: Run GREEN plus YAML/Bash validation**

```bash
.venv/bin/python -m unittest tests.test_connected_corridor_chain_config -v
bash -n scripts/start_connected_corridor_chain.sh \
  scripts/launch_connected_corridor_chain_visualizer.sh \
  scripts/dispatch_connected_corridor_chain_1v3.sh \
  scripts/dispatch_connected_corridor_chain_2v2.sh
```

Expected: all pass.

---

### Task 7: Prove four-robot scheduling and fault behavior in process

**Files:**
- Create: `tests/test_connected_corridor_chain_flow.py`
- Modify generic production files only when a RED test exposes a design defect.

**Interfaces:**
- Consumes: connected-chain YAML through the real Registry, Arbiter, Tracker, Planner, and Task Gate.
- Produces: deterministic proofs for clear direct travel, 1v3, 2v2, dynamic insertion, fairness, and faults.

- [ ] **Step 1: Add deterministic 1v3 and 2v2 RED tests**

Drive telemetry node by node. Record every upstream goal. Assert direct robots
do not receive side-bay goals, yielding robots stop only at side bays, opposite
directions never overlap a block, and all final jobs complete with empty block
resources.

- [ ] **Step 2: Add dynamic and fairness RED tests**

Insert trailing jobs after the first robot enters C1 and after an opposite robot
enters SIDE_2. Assert the current authority is preserved, the overlapping batch
closes, and the oldest opposite direction is eventually admitted. Continue
injecting newer same-direction requests and prove accumulated wait age prevents
starvation of the older opposite request.

- [ ] **Step 3: Add fault RED tests**

Expire telemetry independently in C1, C2, and C3. Assert every unreleased
authority block becomes faulted and no conflicting job is forwarded. Omit
side-bay arrival telemetry and prove opposite traffic remains held. Initialize
the gate with stale or ambiguous occupancy and prove affected blocks remain
fail-closed until fresh telemetry or an explicit safe reset reconciles them.

- [ ] **Step 4: Make only generic fixes and run GREEN**

Do not add scenario IDs to production. Run chain flow, movement authority,
Task Gate, occupancy, staging, and independent multi-corridor tests; expected:
all pass.

- [ ] **Step 5: Run the full suite**

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: at least the current 93 tests plus all new chain tests pass.

---

### Task 8: Validate actual VDA5050 runtime, document, commit, and push

**Files:**
- Create ignored runtime verifier scripts under `.runtime/` only for local evidence.
- Modify: `README.md`
- Modify: `docs/simulation_run_guide.md`

**Interfaces:**
- Consumes: complete connected-chain stack.
- Produces: actual 1v3 and 2v2 acceptance evidence, documented commands, clean commit, and pushed feature branch.

- [ ] **Step 1: Restart only this repository's connected runtime**

Stop the current repository stack, start `start_connected_corridor_chain.sh`,
and verify initial robots occupy distinct endpoint slots, jobs are empty, every
block is `FREE`, and no fault exists.

- [ ] **Step 2: Poll connected 1v3 to completion**

Record log offset, dispatch 1v3, and poll `/traffic/status` until all jobs are
`COMPLETE` or timeout/fault occurs. Verify no planned main-lane stop, side-bay
use only for a yielding authority, correct final nodes, all blocks `FREE`, and
empty block occupants/reservations/waiting.

- [ ] **Step 3: Restart and poll connected 2v2 to completion**

Apply the same acceptance checks. Additionally verify opposite directions run
concurrently only on disjoint C1/C3 intervals and never overlap C2.

- [ ] **Step 4: Inspect in the VDA5050 Visualizer**

Launch the connected map and retain the completed runtime for user inspection.
Confirm all four robot traces are collision-free and no clear-chain robot makes
an unnecessary side-bay visit.

- [ ] **Step 5: Document and verify the final tree**

Document start, visualizer, 1v3, 2v2, status, and log commands. Run focused
tests, the full suite, `git diff --check`, YAML parsing, Bash syntax, and:

```bash
rg -n 'AGV_|CHAIN_|C1|C2|C3' traffic_control
```

Expected: no scenario robot/node/block identifiers in production Python.

- [ ] **Step 6: Commit and push only tracked deliverables**

Explicitly stage production, tests, configuration, scripts, and docs. Confirm
`.runtime` and `recording_260919.jsonl` remain unstaged. Commit with a message
describing connected rolling reservations, push
`feature/p4-single-passing-bay-poc`, and verify local/remote hashes match.
