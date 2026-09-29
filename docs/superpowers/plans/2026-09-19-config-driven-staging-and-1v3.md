# Config-driven Staging and 1v3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add physically distinct, configuration-driven staging slots and validate staging-enabled 2v2 and one-left-to-right-against-three-right-to-left passing-bay scenarios in the real VDA5050 runtime.

**Architecture:** Preserve the existing P4 stack and add a staging-specific map, Simulator configuration, Fleet Adapter configuration, Compose override, Arbiter configuration, and launch fixtures. Reuse the generic `RouteIntent`/`RouteStep` state machine: each configured route begins at a capacity-one physical slot, traverses the existing safe corridor phases, and ends at a distinct capacity-one slot. Generalize only the runtime launcher inputs; admission policy remains driven by holding bays, blocks, direction domains, and route steps rather than robot identities.

**Tech Stack:** Python 3.12, `unittest`, PyYAML, Bash, Docker Compose, Open-RMF, MQTT, VDA5050 Simulator, PyQt5 Visualizer.

**Spec:** `docs/superpowers/specs/2026-09-19-config-driven-staging-and-1v3-design.md`

## Global Constraints

- Work only in `/home/han30230/rmf-work/rmf_passing_bay_poc` on `feature/p4-single-passing-bay-poc`.
- Preserve `recording_260919.jsonl` and every unrelated local change.
- Do not add robot IDs, scenario cardinality, or P4 node IDs to production Python branches.
- Staging slot count, coordinates, capacities, block endpoints, route steps, release nodes, and scenario assignments remain configuration or launch data.
- Keep the existing 1v1, 2v1, and 2v2 stack working.
- Do not widen the existing passing-bay geometry to hide state-classification defects.
- Follow RED-GREEN TDD and record the expected pre-implementation failures.
- Do not commit implementation until focused tests, the full suite, staging 2v2 runtime, and staging 1v3 runtime all pass.
- Never stage `.runtime` files or `recording_260919.jsonl`.

---

### Task 1: Make the existing runtime launcher select configuration through environment data

**Files:**
- Modify: `scripts/start_p4_passing_bay.sh`
- Modify: `tests/test_portable_workspace.py`

**Interfaces:**
- Consumes: existing `PASSING_BAY_DISPATCH_SCRIPT` and positional robot IDs.
- Produces: optional `PASSING_BAY_SIMULATOR_SCENARIO`, `PASSING_BAY_COMPOSE_FILE`, `PASSING_BAY_ARBITER_CONFIG`, and `PASSING_BAY_RUNTIME_NAME` environment variables with current-file defaults.

- [x] **Step 1: Write the failing portability test**

Add assertions that the start script contains all four generic environment
selectors, resolves relative paths from `workspace_dir`, and retains the
current defaults:

```python
def test_start_script_accepts_config_driven_runtime_files(self) -> None:
    script = (ROOT / "scripts/start_p4_passing_bay.sh").read_text(
        encoding="utf-8"
    )
    for name in (
        "PASSING_BAY_SIMULATOR_SCENARIO",
        "PASSING_BAY_COMPOSE_FILE",
        "PASSING_BAY_ARBITER_CONFIG",
        "PASSING_BAY_RUNTIME_NAME",
    ):
        self.assertIn(name, script)
    self.assertIn("p4_scenario.yaml", script)
    self.assertIn("docker-compose.p4-passing-bay.yml", script)
    self.assertIn("corridor_blocks_p4_passing_bay.yaml", script)
```

- [x] **Step 2: Run the test and verify RED**

Run:

```bash
.venv/bin/python -m unittest \
  tests.test_portable_workspace.PortableWorkspaceTests.test_start_script_accepts_config_driven_runtime_files \
  -v
```

Expected: FAIL because the selectors do not exist.

- [x] **Step 3: Implement selector defaults without changing current behavior**

Use repository-relative defaults and normalize relative override values under
`workspace_dir`:

```bash
runtime_name="${PASSING_BAY_RUNTIME_NAME:-p4_passing_bay}"
simulator_source="${PASSING_BAY_SIMULATOR_SCENARIO:-rmf_dev_tool-main/vda5050_robot_simulator/p4_scenario.yaml}"
passing_compose="${PASSING_BAY_COMPOSE_FILE:-rmf_platform-main/docker-compose.p4-passing-bay.yml}"
arbiter_config="${PASSING_BAY_ARBITER_CONFIG:-config/corridor_blocks_p4_passing_bay.yaml}"
simulator_config="${runtime_dir}/${runtime_name}_runtime.yaml"
```

Read the selected Simulator YAML by absolute resolved path in the embedded
Python. Pass the selected Compose and Arbiter files to the existing commands.
Keep PID and main log filenames unchanged so the stop script remains valid.

- [x] **Step 4: Verify GREEN and existing portability checks**

Run:

```bash
.venv/bin/python -m unittest tests.test_portable_workspace -v
```

Expected: all portable-workspace tests pass.

---

### Task 2: Add the six-slot staging map and robot registrations

**Files:**
- Create: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_passing_bay_staging.yaml`
- Create: `rmf_dev_tool-main/vda5050_robot_simulator/p4_staging_scenario.yaml`
- Create: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/config/p4_staging.yaml`
- Create: `rmf_platform-main/docker-compose.p4-passing-bay-staging.yml`
- Create: `tests/test_passing_bay_staging_config.py`

**Interfaces:**
- Consumes: the current `2101..2108` corridor and `6137` side-bay topology.
- Produces: graph nodes `P4_LS1`, `P4_LS2`, `P4_LS3`, `P4_RS1`, `P4_RS2`, `P4_RS3`; five registered simulation robots; a staging Compose override.

- [x] **Step 1: Write failing map and registration tests**

Create `PassingBayStagingConfigurationTests` that loads the four new files and
asserts:

```python
SLOTS = {
    "P4_LS1": (6.833, 96.0),
    "P4_LS2": (3.2, 95.0),
    "P4_LS3": (3.2, 90.7),
    "P4_RS1": (73.7274, 96.0),
    "P4_RS2": (77.3, 95.0),
    "P4_RS3": (77.3, 90.7),
}
```

- every slot name exists exactly once at its distinct coordinate;
- each left slot has bidirectional lanes to `2101`;
- each right slot has bidirectional lanes to `2108`;
- existing main and side-bay lanes are unchanged;
- Simulator robots start at `A1=P4_LS1`, `A2=P4_LS2`, `B1=P4_RS1`,
  `B2=P4_RS2`, and `B3=P4_RS3` coordinates;
- all initial positions are pairwise distinct;
- the Fleet Adapter registers exactly those five robots;
- Compose selects `p4_staging.yaml` and `p4_passing_bay_staging.yaml`.

- [x] **Step 2: Run the new configuration module and verify RED**

Run:

```bash
.venv/bin/python -m unittest tests.test_passing_bay_staging_config -v
```

Expected: ERROR/FAIL because the staging files do not exist.

- [x] **Step 3: Add the staging graph**

Copy the current passing-bay graph and append six named vertices using the
tested coordinates. Add bidirectional lanes from left slots to `2101` and
right slots to `2108`. Preserve graph index metadata and mark only physical
charging locations as chargers when required by the Fleet Adapter.

- [x] **Step 4: Add Simulator and Fleet Adapter configurations**

Use the tested coordinates and point each robot toward its local merge. Add
`AGV_B3` only to the new staging configuration. Keep manufacturer, MQTT
prefix, footprint, vicinity, speed, and reference coordinates consistent with
the current stack.

- [x] **Step 5: Add the staging Compose override and verify GREEN**

Use the current build/launch command with only the config and map paths changed.
Run the new configuration tests; expected: all pass.

---

### Task 3: Describe capacity-one slots and complete slot-to-slot routes in Arbiter YAML

**Files:**
- Create: `config/corridor_blocks_p4_passing_bay_staging.yaml`
- Modify: `tests/test_passing_bay_staging_config.py`

**Interfaces:**
- Consumes: staging graph node names and existing Arbiter semantics.
- Produces: six capacity-one slot holding bays; slot-specific approach,
  crossing, side-bay, and direct blocks; explicit route intents used by 2v2
  and 1v3.

- [x] **Step 1: Add failing Arbiter-configuration assertions**

Assert that:

- six slot holding bays map one-to-one to the six graph slot nodes and have
  capacity one;
- slot circle centers are pairwise distinct and do not contain other slot
  centers;
- every configured block edge exists as a directed graph lane;
- west-advance blocks share `P4_WEST_ADVANCE`;
- crossing, side, and direct blocks share `P4_PASSING_EVENT`;
- forward crossing steps retain `release_node: "2106"`;
- reverse side-bay exit steps retain
  `requires_opposite_routes_cleared: true`;
- B2/B3 each have a via-side fallback and a guarded direct route;
- route endpoints match their physical source and destination slots;
- no robot ID appears anywhere in the Arbiter YAML.

- [x] **Step 2: Run the module and verify RED**

Expected: FAIL because the staging Arbiter file is absent.

- [x] **Step 3: Add slot holding bays and managed blocks**

Use these holding-bay IDs:

```text
HB_LEFT_SLOT_1..3
HB_RIGHT_SLOT_1..3
HB_WEST_GATE
HB_MIDDLE_SIDE
```

Represent an uninterrupted movement from a source slot to its next safe stop
as one block. Do not stop trailing robots at shared `2101` or `2108` merge
points. Each block's graph edges include its staging spur plus the applicable
existing corridor edges. Use configuration geometry around the actual path;
overlap is resolved only by the existing active-grant/source/destination rules.

- [x] **Step 4: Add explicit site-policy routes**

Provide the routes needed by the scenarios:

```text
P4_LS1 -> P4_RS1  (via west gate, release 2106)
P4_LS2 -> P4_RS3  (via west gate, release 2106)
P4_RS1 -> P4_LS1  (via side bay)
P4_RS2 -> P4_LS2  (via side bay + guarded direct alternative)
P4_RS3 -> P4_LS3  (via side bay + guarded direct alternative)
```

The direct alternatives set `requires_no_opposite_jobs: true`. The side-bay
exit alternatives set `requires_opposite_routes_cleared: true`. Route IDs may
describe physical slots but must not include robot IDs.

- [x] **Step 5: Parse the complete registry and verify GREEN**

Add `CorridorRegistry.from_yaml(STAGING_ARBITER_PATH)` to the test so endpoint,
edge, release-node, and holding-bay validation runs through production parsing.
Run the staging configuration module; expected: all pass.

---

### Task 4: Prove staging admission and 1v3 ordering through the real state machine

**Files:**
- Create: `tests/test_passing_bay_staging_flow.py`
- Modify only if a RED test exposes a generic defect:
  `traffic_control/models.py`, `traffic_control/corridor_registry.py`,
  `traffic_control/direction_arbiter.py`, `traffic_control/robot_tracker.py`,
  or `traffic_control/task_gate.py`

**Interfaces:**
- Consumes: `config/corridor_blocks_p4_passing_bay_staging.yaml` and existing
  `TaskGate` APIs.
- Produces: deterministic in-process proofs for distinct-slot 2v2 and 1v3.

- [x] **Step 1: Add shared test helpers**

Load slot and corridor coordinates from the staging config or one local
constant mapping. Reuse a recording forwarder that records `(robot_id,
goal_node)` pairs. Feed telemetry through `RobotTracker.ingest_state()` and
advance only with `TaskGate.tick()`.

- [x] **Step 2: Write the failing capacity/merge test**

Submit two same-side jobs whose first route steps need incompatible resources.
Assert the first step is released, the second stays `WAITING`, and its robot
remains in its configured source slot. Move the first robot to its safe stop,
tick, and assert the second can then progress without a block fault.

- [x] **Step 3: Write the failing staging 2v2 flow**

Drive A1/A2/B1/B2 through their configured nodes and assert:

```text
B1 reaches side bay
A1 clears 2106
A2 is released through the gate
B1 still has no left-slot goal
A2 clears 2106
B1 receives its left-slot goal
B2 switches to the direct route
all four finish at distinct slots
```

Assert all blocks and queues are clean.

- [x] **Step 4: Write the failing staging 1v3 flow**

Submit A1, B1, B2, and B3 in that order. Verify B2/B3 remain in distinct
right slots while B1 uses the side bay. After A1 clears, verify B1 departs;
after the opposite route is terminal, verify B2 then B3 use guarded direct
routes in submission order. Finish at `RS1`, `LS1`, `LS2`, and `LS3` and
assert every job is `COMPLETE`.

- [x] **Step 5: Run both new tests and capture RED**

Run:

```bash
.venv/bin/python -m unittest tests.test_passing_bay_staging_flow -v
```

Expected: failures identify any missing or incorrect configured resource or
generic state transition. A production edit is allowed only for a failing
generic invariant demonstrated by the test.

- [x] **Step 6: Make the smallest generic GREEN correction if required**

Do not add scenario switches. If the existing engine already passes with the
new YAML, make no production Python change. If it fails, express the missing
semantics as a model/config field with a default preserving existing routes,
then test that field with neutral robot and node names before using it in P4.

- [x] **Step 7: Verify new and existing flow tests**

Run:

```bash
.venv/bin/python -m unittest \
  tests.test_block_occupancy \
  tests.test_passing_bay_flow \
  tests.test_passing_bay_staging_flow \
  -v
```

Expected: all pass.

---

### Task 5: Add repository-relative staging launch and dispatch fixtures

**Files:**
- Create: `scripts/start_p4_passing_bay_staging_2v2.sh`
- Create: `scripts/t4_dispatch_passing_bay_staging_2v2.sh`
- Create: `scripts/start_p4_passing_bay_staging_1v3.sh`
- Create: `scripts/t4_dispatch_passing_bay_staging_1v3.sh`
- Create: `scripts/launch_p4_passing_bay_staging_visualizer.sh`
- Modify: `tests/test_passing_bay_staging_config.py`

**Interfaces:**
- Consumes: generic launcher selectors from Task 1 and staging files from Tasks
  2-3.
- Produces: one-command 2v2/1v3 runtimes and the matching Visualizer map.

- [x] **Step 1: Add failing script assertions**

Assert all scripts exist, use `BASH_SOURCE`, contain no workspace absolute
path, and select the staging scenario, Compose, Arbiter, and runtime name.
Parse dispatch calls and assert:

```text
2v2: A1->RS1, A2->RS3, B1->LS1, B2->LS2
1v3: A1->RS1, B1->LS1, B2->LS2, B3->LS3
```

- [x] **Step 2: Run script tests and verify RED**

Expected: FAIL because scripts are missing.

- [x] **Step 3: Implement thin start wrappers**

Each wrapper exports the four runtime selectors and
`PASSING_BAY_DISPATCH_SCRIPT`, then `exec`s the existing generic start script
with its robot list. Keep robot selection in the scenario fixture.

- [x] **Step 4: Implement dispatch and Visualizer wrappers**

Dispatch in the documented order with configurable one-second spacing. The
Visualizer wrapper activates the repository `.venv` Python directly and opens
`p4_passing_bay_staging.yaml` without starting another Simulator.

- [x] **Step 5: Verify GREEN**

Run staging configuration and portable-workspace modules; expected: all pass.

---

### Task 6: Document field substitution and run the full unit suite

**Files:**
- Modify: `README.md`
- Modify: `docs/simulation_run_guide.md`
- Modify: `tests/test_portable_workspace.py`

**Interfaces:**
- Consumes: final script names and behavior.
- Produces: reproducible staging 2v2/1v3 commands and an explicit field
  deployment boundary.

- [x] **Step 1: Add failing documentation assertions**

Assert README and the run guide mention both staging start scripts, both
dispatch scripts, the staging Visualizer script, distinct capacity-one slots,
and that production coordinates must be surveyed.

- [x] **Step 2: Run the documentation test and verify RED**

Expected: FAIL for missing staging documentation.

- [x] **Step 3: Update documentation**

Document commands, expected movement order, initial/final slot assignments,
event checks, and why robot IDs in scenario scripts are fixtures rather than
admission policy. Explain the configuration fields that a real site replaces.

- [x] **Step 4: Run focused tests**

```bash
.venv/bin/python -m unittest \
  tests.test_portable_workspace \
  tests.test_passing_bay_staging_config \
  tests.test_passing_bay_staging_flow \
  tests.test_passing_bay_config \
  tests.test_passing_bay_flow \
  -v
```

- [x] **Step 5: Run the complete suite**

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: more than 69 tests, zero failures.

---

### Task 7: Validate staging 2v2 in the actual VDA5050 runtime

**Files:**
- Runtime-only, ignored: `.runtime/verify_staging_2v2.py`, logs, PID files, and
  final JSON snapshots.

**Interfaces:**
- Consumes: staging 2v2 start/dispatch scripts.
- Produces: condition-polled runtime evidence; no tracked source changes.

- [x] **Step 1: Inspect and stop only this repository's current runtime**

Record `ps`, ports, current status, and PID ownership. Use
`./scripts/stop_p4_passing_bay.sh --all` only after confirming the PIDs belong
to this workspace. Do not stop unrelated GUI or workspace processes.

- [x] **Step 2: Start staging 2v2 and verify initial state**

Start in a persistent terminal, open the staging Visualizer, and assert:

```text
A1=LS1, A2=LS2, B1=RS1, B2=RS2
jobs={}
all blocks FREE
all block queues/reservations empty
fault count=0
```

- [x] **Step 3: Dispatch and poll completion**

Record the log byte offset/time, dispatch once, and poll status until all four
jobs are `COMPLETE` or a bounded overall timeout expires. Do not use a fixed
sleep as the verdict.

- [x] **Step 4: Assert runtime order and final state**

Require A2's release-node event before B1's side-bay exit release, distinct
final slots, all blocks `FREE`, empty block occupants/reservations/waiters,
empty holding-bay reservations, and no run-window faults.

---

### Task 8: Validate staging 1v3 and finish the branch

**Files:**
- Runtime-only, ignored: `.runtime/verify_staging_1v3.py` and snapshots.
- Tracked files: only files listed in Tasks 1-6 and any justified generic
  production correction from Task 4.

**Interfaces:**
- Consumes: staging 1v3 start/dispatch scripts.
- Produces: runtime proof, final implementation commit, and pushed branch.

- [x] **Step 1: Cleanly restart into staging 1v3**

Verify initial `A1=LS1`, `B1=RS1`, `B2=RS2`, `B3=RS3`, empty jobs, free
blocks, and zero faults.

- [x] **Step 2: Dispatch and condition-poll**

Require this event relationship:

```text
B1 reaches SIDE_BAY
A1 ROBOT_CLEARED_BLOCK release_node=2106
B1 receives its left-slot goal
A1 completes at RS1
B2 and B3 switch/release direct in FIFO order
B1/B2/B3 complete at LS1/LS2/LS3
```

- [x] **Step 3: Assert final cleanup**

All four jobs must be `COMPLETE`; robots must be stationary at distinct target
slots; all blocks must be `FREE`; every block occupant/reservation/wait queue
and holding-bay reservation must be empty; run-window `BLOCK_FAULT` and
`ROBOT_FAULT` count must be zero.

- [x] **Step 4: Re-run final verification**

Run the full unit suite again if runtime debugging changed tracked files, then
run `git diff --check`, review the complete diff, and ensure only
`recording_260919.jsonl` remains untracked.

- [x] **Step 5: Commit and push only after both runtimes pass**

Stage the reviewed tracked files explicitly, excluding `.runtime` and the
recording. Commit with a message such as:

```text
Add config-driven staging and validated 1v3 flow
```

Push `feature/p4-single-passing-bay-poc` and verify the remote hash equals the
local hash.
