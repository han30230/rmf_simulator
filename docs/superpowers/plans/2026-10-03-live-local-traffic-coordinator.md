# Live Local Traffic Coordinator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make initial planning and ordinary RMF replans complete the original tasks of dynamically arriving robots through a configured single-lane bidirectional corridor with side bays, then prove the behavior in the actual RMF/VDA5050 simulation.

**Architecture:** Keep the existing planner and negotiation paths outside configured local-traffic regions. Add bounded diagnostics and a generic high-level bay-yield candidate generator that uses RMF planners and schedule validation for every movement and wait segment, preserves final occupancy, and returns complete original-goal plans. A fleet-shared coordinator snapshots active `GoToPlace` requests, solves off the fleet worker, rejects stale generations, and applies only a fully validated group result through the same `ExecutePlan` path used by normal RMF planning.

**Tech Stack:** C++17, ROS 2 Jazzy, Open-RMF `rmf_traffic` and `rmf_ros2`, rxcpp, Docker Compose, Python 3/unittest, YAML, VDA5050 MQTT simulator.

**Spec:** `../specs/2026-10-02-local-traffic.md`

## Global Constraints

- Do not hardcode robot names, four-robot cardinality, or current waypoint IDs in production C++ or Python.
- Preserve every active event's original goal; reaching a bay is an intermediate stage and never task completion.
- Include moving occupancy, waits, destination occupancy, fixed idle robots, and external schedule participants in validation.
- Never interpret TIMEOUT as proof that no solution exists and never execute an unvalidated greedy fallback as a group solution.
- The solver must run off the fleet worker with a bounded cancellation/deadline mechanism that leaves no orphan worker.
- Revalidate robot position, goal, request generation, and relevant schedule state before applying a result; reject stale results.
- Group application must be all-or-recover: a partial application cannot release another robot into the affected corridor.
- Use actual progress/dependency acknowledgement for release; do not use fixed sleeps or predicted arrival alone.
- Keep the feature opt-in and disabled by default. Preserve the external arbiter only as a baseline, never as evidence for core success.
- Record source commits, image IDs, loaded executable/library paths, configured budgets, task timing, trajectories, and known limitations.

---

### Task 1: Reproducible environment, baseline, and search diagnostics

**Files:**
- Create: `core_local_traffic/results/2026-10-03/environment.yaml`
- Create: `core_local_traffic/results/2026-10-03/baseline-summary.yaml`
- Create: `core_local_traffic/results/2026-10-03/commands.md`
- Modify: `core_local_traffic/evaluator.cpp`
- Modify: `core_local_traffic/patches/rmf_traffic-negotiation-cancellation.patch`
- Modify: `core_local_traffic/run_matrix.py`
- Test: `tests/test_core_evaluator.py`
- Test: `tests/test_core_matrix.py`

**Interfaces:**
- Consumes: pinned revisions in `core_local_traffic/manifest.yaml` and scenarios under `core_local_traffic/scenarios/`.
- Produces: evaluator YAML `diagnostics` with per-table/respond/plan/rollout counters, elapsed time, candidate endpoints, termination reason, configured limits, and explicit `TIMEOUT`, `SATURATED`, `COST_LIMIT`, `NO_ROUTE`, or `NO_PROPOSAL` status.

- [ ] Add a failing evaluator test that requires termination diagnostics and proves a timeout result does not contain `physical_impossibility: true`.
- [ ] Add a failing matrix-runner test that requires binary SHA256, loaded `librmf_traffic.so` path/SHA256, scenario SHA256, and exact limits in each result.
- [ ] Run both tests against the baseline and record their expected missing-field failures.
- [ ] Add bounded counters/timers around centralized table selection, SimpleNegotiator response planning and rollout expansion without changing candidate choice.
- [ ] Propagate planner interrupted/saturated/cost-limit state into the evaluator output and distinguish subprocess hard timeout.
- [ ] Record host ROS, Docker version, image ID, image RMF prefixes, entrypoint, source revisions, baseline library hashes, and the exact commands already reproduced on 2026-10-03.
- [ ] Rebuild the pinned evaluator and rerun one-, two-, three-, and four-agent baselines, including the 60-second three-agent case.
- [ ] Run focused tests and commit only diagnostics, tests, and reproducibility evidence.

### Task 2: Configured bay-yield candidate generation and complete-plan validation

**Files:**
- Create: `core_local_traffic/BayYieldSolver.hpp`
- Create: `core_local_traffic/BayYieldSolver.cpp`
- Modify: `core_local_traffic/CMakeLists.txt`
- Modify: `core_local_traffic/evaluator.cpp`
- Create: `core_local_traffic/scenarios/online_ab_c.yaml`
- Create: `core_local_traffic/scenarios/online_ab_c_d.yaml`
- Test: `tests/test_bay_yield_solver.py`
- Test: `tests/test_core_evaluator.py`

**Interfaces:**
- Consumes: named graph topology, holding/passthrough flags, robot profile/traits, starts, original goals, fixed occupancy, schedule viewer, and bounded `SolveOptions`.
- Produces: `SolveResult{status, generation, plans_by_participant, stage_trace, dependencies, metrics}` where every plan ends at the registered original goal and remains occupied through the common horizon.

- [ ] Add failing tests for the unchanged 3-agent and 4-agent inputs that require `VALID`, original goals, no pairwise trajectory conflict, no dependency cycle, and no disappearance after arrival.
- [ ] Add failing tests for overlapping starts, fixed bay/exit blockers, missing return route, passthrough waiting, impossible no-bay input, and cancellation/deadline.
- [ ] Implement topology discovery from configured holding/passthrough waypoints and graph connectivity; do not embed `6137` or `210x` in solver code.
- [ ] Enumerate both yield directions, reachable bays, safe endpoint/holding waits, and bounded nonmonotone retreat candidates needed for multiple waves.
- [ ] Use RMF planner results for each segment, insert explicit stationary trajectories for waits/final occupancy, and validate all pairwise trajectories and external schedule constraints.
- [ ] Reject stage graphs with cyclic progress dependencies and require the final stage of every movable participant to reach its original goal.
- [ ] Record candidate counts, prune reasons, expanded states, solve p50/p95/max inputs, and the exact increase from baseline limits.
- [ ] Run RED/GREEN tests, the unchanged baseline matrix, and commit the standalone solver milestone.

### Task 3: Fleet-shared asynchronous LocalTrafficCoordinator

**Files:**
- Add to patch: `rmf_fleet_adapter/src/rmf_fleet_adapter/local_traffic/LocalTrafficCoordinator.hpp`
- Add to patch: `rmf_fleet_adapter/src/rmf_fleet_adapter/local_traffic/LocalTrafficCoordinator.cpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/agv/FleetUpdateHandle.cpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/agv/RobotContext.hpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/events/GoToPlace.cpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/CMakeLists.txt`
- Test through patch: `rmf_fleet_adapter/test/unit/local_traffic/test_LocalTrafficCoordinator.cpp`
- Test: `tests/test_core_overlay.py`

**Interfaces:**
- Produces: fleet-owned `LocalTrafficCoordinator::request_plan(Request)` returning a cancellable future/result keyed by `{participant_id, request_id, generation}`.
- `Request` contains immutable original goal, current starts, profile, participant plan/version, active command state, and configured region ID.
- `Result` contains complete plans for all affected requests plus the snapshot/generation used to compute them.

- [ ] Add failing upstream C++ tests for one shared coordinator per fleet, worker non-blocking behavior, generation increments, stale position/goal/schedule rejection, cancellation, timeout, and fixed idle participants.
- [ ] Add a failing test where C arrives during A/B planning and requires the affected group to be resnapshotted without changing original goals.
- [ ] Add a failing test where one apply callback rejects and require all corridor members to remain held/recovering with no authority release.
- [ ] Implement worker-owned registration/snapshot collection and a dedicated bounded executor for solve work.
- [ ] Return results to the fleet worker, revalidate relevant fields, and discard stale results with explicit reason metrics.
- [ ] Implement prepare/apply acknowledgement and fail-closed partial-apply recovery using safe current plans or an explicit unrecoverable state.
- [ ] Register/clear active movement requests on completion, cancellation, interruption, goal change, and task replacement.
- [ ] Run patch application tests and upstream coordinator unit tests, then commit.

### Task 4: GoToPlace, ordinary replan, and negotiation integration

**Files:**
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/events/GoToPlace.cpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/events/ExecutePlan.cpp`
- Modify patch target: `rmf_fleet_adapter/src/rmf_fleet_adapter/Negotiator.cpp`
- Modify: `core_local_traffic/patches/rmf_ros2-search-budgets.patch`
- Test through patch: `rmf_fleet_adapter/test/unit/local_traffic/test_GoToPlaceLocalTraffic.cpp`
- Test through patch: `rmf_fleet_adapter/test/unit/local_traffic/test_LocalTrafficNegotiation.cpp`
- Test: `tests/test_core_overlay.py`

**Interfaces:**
- Consumes: coordinator from Task 3 after `_chosen_goal` is fixed.
- Produces: conditionally selected complete `agv::Plan` delivered to existing `_execute_plan`; `_respond` defers to matching current group generations while preserving external negotiations.

- [ ] Add failing tests showing initial GoToPlace and `RobotUpdateHandle::replan()` both use the coordinator inside a configured region and normal `FindPath` outside it.
- [ ] Add a failing test proving a bay stage does not finish the GoToPlace event and the final plan reaches the original goal.
- [ ] Add failing tests for stale result, task cancellation, goal change, repeated replan, group-external negotiation, and deprecated dependency handling.
- [ ] Branch in `_find_plan()` only after goal selection; retain existing FindPath behavior when disabled/outside the region or when a validated prior plan must continue.
- [ ] Apply the complete plan through existing `ExecutePlan` so schedule itinerary, dependencies, and robot commands use the same plan.
- [ ] Coordinate `_respond()` with active group generations without globally disabling negotiation or approving stale proposals.
- [ ] Log request/participant/generation, original goal, plan ID, command generation, solve/apply timing, stale reason, and fallback decision.
- [ ] Build and run upstream unit/integration tests and commit.

### Task 5: Exact-source ROS 2 Docker overlay and controlled runtime

**Files:**
- Create: `core_local_traffic/build_ros_overlay.sh`
- Create: `rmf_platform-main/docker-compose.local-traffic-core.yml`
- Create: `scripts/start_local_traffic_core.sh`
- Create: `scripts/stop_local_traffic_core.sh`
- Create: `scripts/verify_local_traffic_runtime.py`
- Test: `tests/test_local_traffic_runtime_scripts.py`
- Modify: `core_local_traffic/README.md`

**Interfaces:**
- Produces: isolated runtime overlay containing patched pinned `rmf_traffic`, patched pinned `rmf_fleet_adapter`, and the repository VDA5050 adapter, with a generated provenance manifest and no changes to the baseline compose files.

- [ ] Add failing script tests requiring exact source revisions, clean/applicable patches, unique overlay path, feature flag default off, no external arbiter, and process-specific overlay sourcing.
- [ ] Build patched `rmf_traffic` and `rmf_fleet_adapter` against the actual Jazzy image dependencies in an isolated volume/directory.
- [ ] Record compiler, build command, source commits, patch hashes, install prefix, executable paths, `LD_DEBUG`/`ldd` evidence, and shared-library hashes.
- [ ] Add a compose override that mounts only experimental map/config/overlay and starts schedule, dispatcher, API, patched fleet adapter, MQTT, and simulator without the Task Gate.
- [ ] Add condition-based readiness and scoped cleanup that cannot terminate unrelated project processes.
- [ ] Verify a one-robot regression and a static two-robot bay exchange before enabling 3/4-agent dynamic scenarios.
- [ ] Commit the reproducible runtime overlay tooling and provenance.

### Task 6: Dynamic online simulation, replan, fault matrix, and final evidence

**Files:**
- Create: `core_local_traffic/runtime_scenarios.yaml`
- Create: `scripts/run_local_traffic_dynamic.py`
- Create: `core_local_traffic/results/2026-10-03/dynamic-summary.yaml`
- Create: `core_local_traffic/results/2026-10-03/known-limitations.md`
- Modify: `core_local_traffic/README.md`
- Test: `tests/test_local_traffic_dynamic_runner.py`

**Interfaces:**
- Consumes: direct RMF API task submission, `/local_traffic/replan`, MQTT state/command traces, schedule/adapter diagnostics, and simulator positions.
- Produces: per-run JSONL/YAML evidence and aggregate completion/collision/wait/replan/solve/stale metrics.

- [ ] Add failing runner tests requiring condition-based task injection, original task identity tracking, actual replan plan/command evidence, trajectory minimum-separation calculation, and explicit incomplete verdicts.
- [ ] Run one robot and 1v1 regressions with the core feature enabled and external arbiter absent.
- [ ] Run A/B active then C during bay approach; A in bay/B passing then C; and C waiting then D, including 2v2, 1v3, and same-direction arrivals.
- [ ] Trigger actual `RobotUpdateHandle::replan()` in each relevant phase and verify a new plan ID/order command, movement, and original task completion.
- [ ] Run speed difference, movement delay, pause, state-report delay, idle bay/exit blocker, task cancel, and goal-change cases.
- [ ] Repeat the three core online scenarios at least 10 times with varied injection timing/seed when supported; record any reduced count and unverified scope.
- [ ] Verify zero physical overlaps, profile-based minimum separation, bounded waiting/no starvation, all original tasks complete, and no stale/partial result was executed.
- [ ] Run full tests, core builds, planner matrix, runtime matrix, independent final review, and commit the evidence and limitations.

