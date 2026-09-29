# P4 Single Passing Bay PoC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Implement and verify the two-robot P4 passing-bay PoC described in the approved design, without changing the existing P4 baseline runtime files.

**Architecture:** Add a dedicated navigation graph, arbiter configuration, Docker Compose override, and dispatch helper. The existing `TaskGate` splits each managed route into safe-node tasks, while three logical passing blocks share one direction domain so B1 yields in side bay `6137` and A1 crosses before B1 rejoins. Because those logical blocks overlap physically, telemetry matching will prefer the block currently granted to that robot.

**Tech Stack:** Python 3, `unittest`, PyYAML, FastAPI runtime, Open-RMF/VDA5050 YAML, Docker Compose, Bash.

**Spec:** `docs/superpowers/specs/2026-09-16-p4-single-passing-bay-poc-design.md`

## Global Constraints

- Keep the existing P4 map, arbiter config, and Compose override unchanged.
- Treat `6137` as an off-line holding bay with capacity 1.
- Keep `unmatched_route_policy: BLOCKED` and telemetry fail-closed behavior.
- Scope runtime proof to A1 and B1; do not dispatch B2.
- Add tests before implementation and observe their expected failure.

---

### Task 1: Add failing passing-bay configuration and flow tests

**Files:**
- Create: `tests/test_passing_bay_config.py`
- Create: `tests/test_passing_bay_flow.py`
- Modify: `tests/test_block_occupancy.py`

**Step 1: Write configuration contract tests**

Cover the new map node/lane topology, side-bay separation, shared direction domain, two-step routes, Compose override targets, and two-robot dispatch helper.

**Step 2: Write the end-to-end gate state-machine test**

Drive A1 and B1 through safe-node telemetry and assert the exact forwarded goal sequence:

```text
A1 -> 2104
B1 -> 6137
A1 -> 2108
B1 -> 2101
```

Also assert B1 waits in `HB_MIDDLE_SIDE`, final jobs are complete, and all blocks/queues/reservations are clear.

**Step 3: Write overlapping-geometry ownership test**

Create two overlapping blocks and verify telemetry selects the block granted to the robot instead of the first block in configuration order.

**Step 4: Run the new tests and confirm RED**

Run:

```bash
python -m unittest -v \
  tests.test_passing_bay_config \
  tests.test_passing_bay_flow \
  tests.test_block_occupancy
```

Expected: failure because passing-bay files and granted-block telemetry disambiguation do not exist yet.

### Task 2: Implement granted-block telemetry disambiguation

**Files:**
- Modify: `traffic_control/corridor_registry.py`
- Modify: `traffic_control/direction_arbiter.py`
- Modify: `traffic_control/robot_tracker.py`

**Step 1: Expose all geometry matches**

Add `blocks_for_position(x, y)` while preserving the legacy single-result method.

**Step 2: Expose a robot's active/granted block**

Add a thread-safe arbiter query that returns the robot's single grant or occupied block.

**Step 3: Prefer the granted block in telemetry**

When multiple block geometries contain the same position, prefer the current block, then the robot's granted block. Keep unexpected unreserved occupancy fail-closed.

**Step 4: Run the focused occupancy test**

Run:

```bash
python -m unittest -v tests.test_block_occupancy
```

Expected: PASS.

### Task 3: Add passing-bay runtime artifacts

**Files:**
- Create: `rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_passing_bay.yaml`
- Create: `config/corridor_blocks_p4_passing_bay.yaml`
- Create: `rmf_platform-main/docker-compose.p4-passing-bay.yml`
- Create: `scripts/t4_dispatch_passing_bay.sh`

**Step 1: Add graph node and lanes**

Copy the baseline P4 graph, add `6137` at `(42.508, 91.100)`, and add bidirectional `2105 <-> 6137` lanes.

**Step 2: Add holding bays, blocks, and routes**

Define `HB_LEFT`, `HB_WEST_GATE`, `HB_MIDDLE_SIDE`, and `HB_RIGHT`; define independent `P4_WEST_ADVANCE`; define the three `P4_PASSING_EVENT` blocks; define the two two-step routes.

**Step 3: Add runtime selectors**

Create a Compose override that uses the existing fleet config and new map. Create a dispatch helper that submits A1 first, waits one configurable second, then submits B1, with B2 excluded.

**Step 4: Run passing-bay tests**

Run:

```bash
python -m unittest -v \
  tests.test_passing_bay_config \
  tests.test_passing_bay_flow
```

Expected: PASS.

### Task 4: Document operation and verify the repository

**Files:**
- Modify: `README.md`

**Step 1: Add concise run commands and expected sequence**

Document the new Compose override, arbiter config, dispatch helper, expected forwarded goals, and two-robot/simulation-only limits.

**Step 2: Run the full regression suite**

Run:

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
python -m compileall -q traffic_control tests
bash -n scripts/t4_dispatch_passing_bay.sh
```

Expected: all tests pass; compile and shell syntax checks succeed.

**Step 3: Review the diff against the design**

Check safety invariants, YAML route endpoint consistency, no credentials, no placeholders, and no edits to existing baseline runtime artifacts.

**Step 4: Publish on a feature branch**

Create a GitHub feature branch from `main`, commit the verified tree, and verify remote paths and commit metadata before reporting completion.
