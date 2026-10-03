# Local MAPF online integration experiment

Date: 2026-10-03/04

This directory makes the experiment transferable without changing either
upstream remote. It contains the complete `rmf_traffic` patch series based on
`e569de3`, the complete `rmf_ros2` patch series based on `e0360cc`, Docker/MQTT
fixtures, compressed simulator telemetry, and the observed results. No physical
robot was used.

The implementation is opt-in (`RMF_LOCAL_MAPF_ENABLED=0` unless explicitly
enabled). It is a bounded proof of concept, not a production-ready traffic
guarantee.

## Topology boundary

| Case | Starts | Original goals | Holding | Terminal occupancy |
|---|---|---|---|---|
| Generic core PASS map | generated staging/queue slots outside the shared narrow segment | generated destination slots outside the shared narrow segment | one graph-declared side holding branch | completed agents do not occupy the only route to another goal |
| P4 online failure map | west: `PARK_W`/`2101`; east: `PARK_E`/`2108` | opposing tasks end at `2101` or `2108` | side bay `6137`, branching at `2105` | `2101` and `2108` are inline endpoints and remain occupied; parking/relocation is not synthesized |
| Earlier fixed-2108 failure | moving agent heads to `2108`; another participant is stationary there | `2108` | `6137` is reachable | persistent destination occupancy physically blocks the goal, correctly producing no proposal |

The generic core tests therefore establish feasibility and reduced negotiation
table growth on a feasible topology. They do not establish that the P4 online
case is solved. The fixed-destination case remains a parking/relocation policy
problem; no occupancy is deleted and no idle robot is moved.

## Integration boundary

`GoToPlace::Active::_find_plan()` registers only the currently active movement
intent: current `StartSet`, original goal, planner, participant ID, request
generation, material movement revision, and report time. Future queued tasks are
not registered as current goals. When 2--4 same-process intents exist, the
shared fleet coordinator:

1. requests stop without blocking the fleet worker;
2. waits for fresh repeated pose reports and pose stability;
3. publishes stationary occupancy while stopped;
4. solves with `rmf_traffic::agv::LocalConflictResolver` off the fleet worker;
5. rejects generation, goal, cancellation, material-position, stale-report,
   past-start, or fresh-conflict mismatches;
6. registers every schedule itinerary, then starts each `ExecutePlan` using the
   already-registered policy;
7. stops the group and restores stationary occupancy on partial execution
   failure, then returns control to legacy RMF planning.

The `_respond()` negotiation path invalidates an active local cycle, requests
stop, restores occupancy, and transfers ownership to legacy negotiation. The
local solver is limited to four agents by both configuration and the core hard
limit. More agents, unsupported inputs, exceptions, budget exhaustion, stale
results, or external conflicts fall back to existing RMF behavior. “Fallback”
means control is returned; it is not a deadlock or safety guarantee.

One failed local solve is now suppressed for an unchanged participant/goal set.
The suppression key is captured when the cycle starts, so a task arriving while
an older set is quiescing creates a new eligible set. This prevents repeated
2/10-second solver calls from amplifying a legacy deadlock.

## Core A/B result

All six generic core cases (`1v1`, dynamic same-direction third, dynamic
opposite-direction third, `2v1`, `1v2`, `2v2`) returned collision-free normal
`map<ParticipantId, Plan>` proposals and completed their original goals. Local
solve times were 0.0036--0.0279 s with 1--2 high-level nodes and 5--12 low-level
replans. The local path used zero negotiation tables/rollout alternatives; the
baseline used 2--5 tables. Baseline also passed those feasible cases and usually
had lower makespan, so this is a table-count/performance improvement rather than
a newly solved baseline failure.

The inline-terminal 2v2 control remained unsolved after 30 s: 77 tables and
1,793 rollout alternatives. Its terminal occupancy is physically incompatible.

## Actual Docker/MQTT result

| Run | Budget | 2-agent result | Dynamic 3-agent | Dynamic 4-agent | Final tasks | Min center distance | Process |
|---|---:|---|---|---|---:|---:|---|
| run2 | 2 s | PASS local solve, holding macro selected | timeout: 23 low-level replans | timeout; repeated retries reached 93 replans before suppression patch | 0/4 | 1.4938 m | no termination |
| run4 | 10 s | PASS twice, holding macro selected | timeout: 67 replans, 20 nodes | timeout: 68 replans, 24 nodes | 0/4 | 1.4938 m | no termination |

The 10-second result rejects the hypothesis that increasing only the time/cost
budget solves the online case. The three/four-agent search still exhausted the
time budget, selected no holding path, and was stale by generation when it
returned. After fallback, A1 and B1 stopped face-to-face on the main line and
the simulator emitted `Robot stopped by detected obstacle`; B2 stopped behind
B1. No robot reached `6137` in either recorded dynamic run. These are FAIL
results: collision avoidance at the command layer and process survival do not
equal task completion.

The initial pre-instrumentation run terminated once with an uncaught
`std::out_of_range` from the experimental asynchronous boundary. The coordinator
now catches solver exceptions and reports an unsupported fallback. Two later
instrumented runs had zero process terminations. The original throw site was not
captured with a stack trace, so the exception boundary is verified but the exact
origin of that first throw remains unproven.

## Verification completed

- `rmf_traffic`: all 99 test cases / 57,886 assertions passed; local MAPF A/B
  subset passed 76 assertions.
- New adapter units: RobotObservation 17 assertions, GroupPlanApplication 20,
  and latest LocalJointPlanningCoordinator 47 assertions / 13 cases.
- `librmf_fleet_adapter.so` and all normal adapter runtime objects linked. The
  aggregate legacy Catch target still fails to link because the old ament test
  setup resolves a different Catch2 ABI; standalone new tests pass.
- Actual MQTT pose telemetry and per-robot simulator logs are under `results/`.
- Schedule registration and execution-start failure injection are covered by
  the pure staged-application unit tests, not by an actual MQTT run.

Not completed as actual Docker/MQTT scenarios: task cancellation, goal change,
deliberate report loss/delay, deliberate speed mismatch/pause, and injected
schedule/command partial failure. They must not be reported as verified. The
generic update handle also exposes pose reports but not a portable driving or
velocity field, so quiescence currently uses fresh pose stability; VDA state
velocity was measured only by the external monitor.

## Reproduction

Apply patches in separate matching repositories; do not combine histories:

```bash
git -C /path/to/rmf_traffic am \
  /path/to/rmf_simulator/experiments/local-mapf-online/core-patches/*.patch
git -C /path/to/rmf_ros2 am \
  /path/to/rmf_simulator/experiments/local-mapf-online/ros2-patches/*.patch
```

Build the core and fleet adapter against the same RMF 3.8 ABI used by the
`rmf-core:latest` image. The lab used:

```bash
cmake --build build/traffic-tests -j2
build/traffic-tests/test_rmf_traffic '[local_mapf]'
RMF_SINGLE_LANE_TIMEOUT_SECONDS=8 \
  build/traffic-tests/test_rmf_traffic '[local_mapf_ab]'

cmake --build build-fleet-only/rmf_fleet_adapter \
  --target rmf_fleet_adapter -j2
cmake --install build-fleet-only/rmf_fleet_adapter
```

Start the RMF services from the simulator repository:

```bash
docker compose \
  -f rmf_platform-main/docker-compose.yml \
  -f rmf_platform-main/docker-compose.p4.yml \
  -f experiments/local-mapf-online/docker-compose.local-mapf.yml \
  up -d rmf_traffic_schedule rmf_traffic_blockade \
  rmf_task_dispatcher vda5050_fleet_adapter
```

Set `RMF_LOCAL_MAPF_MAX_SOLVE_MS=10000` before `docker compose` to reproduce the
10-second control; the default fixture remains 2,000 ms. Launch the supplied
simulator fixture and `monitor.py`, then dispatch in order:

```bash
bash scripts/t4_dispatch_patrol.sh AGV_A1 2108
bash scripts/t4_dispatch_patrol.sh AGV_B1 2101
bash scripts/t4_dispatch_patrol.sh AGV_B2 2101
bash scripts/t4_dispatch_patrol.sh AGV_A2 2108
```

The robot names and P4 waypoints exist only in these simulation fixtures and
dispatch commands. Neither the core resolver nor adapter integration contains
those identifiers, corridor IDs, cardinal directions, or a robot-count-specific
policy.
