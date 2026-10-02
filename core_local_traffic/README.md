# RMF core local traffic laboratory

**This branch is a planning laboratory milestone. It does not yet implement the shared live LocalTrafficCoordinator or demonstrate dynamic RMF/VDA5050 completion.** The original direction arbiter remains the default baseline.

The uploaded implementation specification is preserved in [the spec](../docs/superpowers/specs/2026-10-02-local-traffic.md). The repository snapshot contains a reconstructed Python fleet adapter but no original `rmf_core` checkout. This experiment uses the exact upstream commits in `manifest.yaml` in a separate workspace; it does not upgrade an installed fleet.

## What is implemented

- Separate P4 experimental graph: side bay `6137`, holding flag, two directed connections and parking extensions. This is a synthetic simulation graph, not a surveyed factory map. Existing P4 files are untouched. Initial robots are separated; obstacle detection remains enabled.
- C++ evaluator using real `rmf_traffic::agv::CentralizedNegotiation`, `Planner`, `Database` and registered participants. No Python MAPF substitute. Original goals are kept. It validates initial occupancy, all trajectory pairs without masking collisions using dependencies, and continued goal occupancy through a common finite horizon (latest arrival + 60 seconds by default). `VALID` means planner-only geometric/time validation, not execution safety or deadlock freedom.
- Exact-version patches for ROS core search budgets/cancellation and traffic negotiation cancellation. The evaluator was compiled and executed against patched traffic. The ROS fleet-adapter patch has been application-checked but **has not been compiled against ROS**.
- Opt-in ROS `/local_traffic/replan` input calls upstream `EasyRobotUpdateHandle.more().replan()`. It requests an actual core replan rather than merely redrawing or mutating tasks. The Python test verifies forwarding; it is not evidence of a running ROS replan.
- `RMF_NAVIGATE`, `RMF_STOP`, `RMF_REPLAN_REQUEST` logging. No new adapter movement gating.

## Measured results

Host: Ubuntu 24.04; C++ compiler GCC 13.3; Python 3.12. No ROS, colcon, Docker or `/opt/ros` installation. Actual runtime preflight is `BLOCKED` (see `results/runtime_preflight.yaml`).

| Case | Result | Repetitions / budget |
|---|---|---|
| Single robot | VALID, original goal | integration test |
| Opposite 2 robots + bay | VALID; bay used; both goals preserved | 3/3, about 2.9–3.2 seconds |
| 3 robots from separated starts | TIMEOUT | 3/3 with 10-second budget |
| 4 robots from separated starts | TIMEOUT | 3/3 with 10-second budget |
| 3 robots, longer budget | TIMEOUT | 1 run, 60.17 seconds |
| Overlapping starts | INVALID_INITIAL_OCCUPANCY | integration test |
| Fixed idle robot at exit | no false VALID | integration test |
| Opposite pair without bay | no false VALID | integration test |

Raw results are in `results/`. We reduced the spec's 10 repeats to 3 for the initial static matrix; these repeats use identical deterministic snapshots and are not independent randomized online trials. No dynamic A/B→C→D task sequence has run. TIMEOUT does **not** establish physical impossibility. Increasing budgets alone has not established the requested 3/4-robot behavior. The backend is still the existing RMF negotiation algorithm, with its table-version pruning and rollout horizon; it is not CBS/ECBS.

The unpatched 3-agent run exceeded an external 25-second timeout despite a 10-second planner interrupter. Source tracing found `SimpleNegotiator` calls `options.interrupt_flag(nullptr)` before rollout; that setter also clears the interrupter callback. The traffic patch preserves callbacks for rollouts/validator loops and checks them between centralized tables. After rebuild, 10-second runs returned TIMEOUT around 10.01–10.02 seconds. A subprocess watchdog in `run_matrix.py` kills and reaps the whole process if needed. This watchdog is suitable for the laboratory executable; it is not a live fleet executor.

## Build and reproduce the planner checks

On Ubuntu 24.04 install development tools in your lab environment:

```bash
sudo apt-get update
sudo apt-get install git cmake g++ libeigen3-dev libccd-dev libyaml-cpp-dev python3-yaml
bash core_local_traffic/build_lab.sh /tmp/rmf-local-lab
export RMF_LOCAL_TRAFFIC_EVALUATOR=/tmp/rmf-local-lab/build/evaluator/local_traffic_evaluator
python3 -m unittest discover -s tests -v
python3 core_local_traffic/run_matrix.py --binary "$RMF_LOCAL_TRAFFIC_EVALUATOR" \
  --output /tmp/local-traffic-results --repeat 10
```

`build_lab.sh` requires a new directory and clones pinned upstream sources. If you already have clean copies, `apply_overlay.py DIRECTORY` first checks both revisions and patch applicability; `--apply` writes the changes. Dirty or mismatching repositories are rejected. Re-running against an already patched tree is intentionally rejected. Header `SearchBudget.hpp` is installed alongside the ROS patch by that script.

The experimental graph parser supports one level and metadata-only `graph_idx` lane properties. Other lane semantics are rejected rather than silently lost. Robot shape is a conservative 0.7 m circle (fleet footprint is 0.5 m and vicinity 0.7 m); speed/acceleration and non-reversible differential steering match the supplied fleet configuration. It does not model unknown plant geometry or simulator-specific braking. Inputs are stopped on graph vertices; it does not currently collect moving off-graph `StartSet`s, apply dependencies or integrate schedule snapshots from a running fleet. Fixed external occupancy is modeled explicitly.

## ROS core patch scope and rebuild

`rmf_ros2-search-budgets.patch` changes:

- `GoToPlace::_find_plan`: internal solve budget and longer outer timer when opted in. Ordinary GoToPlace replans share this entry point.
- `SearchForPath`: opt-in compliant cost `max(factor * base_cost, base_cost + extra_cost)` both initially and on resume; node budget passed to greedy and compliant options. Fix the repeated greedy interrupter assignment so compliant gets cancellation/deadline, and handle absent deadlines correctly.
- `Planning`: preserve an explicit saturation limit only when budgets are enabled; otherwise retain upstream's forced 10,000-node limit. With opt-in enabled, other Planning call sites in the same adapter process also honor explicit limits (or use the configured default).
- `FindPath`: log compliant/greedy selection and greedy failure flags. Greedy fallback remains existing behavior and must not be treated as a collision-free group solution.

Budgets are **process-scoped opt-in laboratory settings**, not corridor-scoped policies. They do not automatically increase the separate `services::Negotiate` budget or task allocation search. The live coordinator and request/plan generation logging specified in the MD remain unimplemented.

```bash
export RMF_LOCAL_TRAFFIC_BUDGETS=1
export RMF_LOCAL_TRAFFIC_COST_LEEWAY=10
export RMF_LOCAL_TRAFFIC_EXTRA_COST=120
export RMF_LOCAL_TRAFFIC_NODE_LIMIT=100000
export RMF_LOCAL_TRAFFIC_SOLVE_SECONDS=20
```

Use a separate ROS 2 Jazzy overlay with all dependencies available. Add the patched `rmf_ros2`, `rmf_traffic` and matching RMF dependencies to its `src`, then run `rosdep install --from-paths src --ignore-src -r -y` and `colcon build --packages-up-to rmf_fleet_adapter rmf_fleet_adapter_python`. Source that overlay's `install/setup.bash` in every RMF process/container that loads the adapter library. This is a recipe requiring validation, not a recorded build success. Existing Dockerfile has corporate CA/proxy assumptions and its `rmf-core:latest` image can hide an older binary; record image ID and loaded library paths before comparing runs.

## Real replan test setup (not executed here)

First bypass the existing task arbiter for this experiment, run the schedule/core and the reconstructed adapter with `map/p4_local_traffic.yaml`, `config/p4_local_traffic.yaml` and `--enable-replan-test-control`. Run the VDA5050 simulator with its `p4_local_traffic.yaml`; ensure broker host/name resolution matches your installation. The topic is simulation-only and must not be enabled on a real fleet.

```bash
ros2 topic pub --once /local_traffic/replan std_msgs/msg/String "{data: AGV_A1}"
```

Dispatch to the core API directly, then request replans during bay approach, while inside the bay and while another robot passes. Acknowledgement of `RMF_REPLAN_REQUEST` is insufficient: verify `_find_plan` logs, new plan IDs, command IDs/order IDs, stop acknowledgement, continued physical occupancy, original task completion and actual simulator pose traces. The reconstructed adapter recomputes a graph path between each EasyFullControl destination and its reported start; it does not receive a custom `waypoint_names` sequence from the upstream Destination API. Check that its intermediate commands preserve the planned bay/hold transitions. Its stop callback currently pauses; the simulator unpauses on a new order, which is not proof of the same behavior on real hardware.

## Remaining implementation gate

Do not claim this branch solves the requested online deadlock. Before connecting a group backend into `FleetUpdateHandle`/`RobotContext` and `GoToPlace::_find_plan`/`_respond`, obtain valid 3/4-robot plans or implement and validate full bay-yield/return candidates. Then implement worker-owned snapshots, per-request generations, stale result rejection, group application and partial-apply recovery, preserved goals and dependency-cycle checks. A later runtime must test all MD scenarios, including incoming C/D, delays, idle blockers, cancel/goal change and repeated replans. Currently there is no atomic group-apply protocol, no live safety fallback and no dynamic acceptance evidence.
