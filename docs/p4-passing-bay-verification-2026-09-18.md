# Source holding-bay transition fix: runtime verification

Date: 2026-09-18 (KST). Result: **PASS**.

## Workspace

- `/home/han30230/rmf-work/rmf_passing_bay_poc`
- Branch: `feature/p4-single-passing-bay-poc`
- Base HEAD: `a7b73f0 Validate semantic passing-bay geometry overlap`
- Python: repository `.venv/bin/python`
- No alternate checkout, remote snapshot, or geometry change was used.

## Root cause and change

`CorridorRegistry.from_dict()` normalizes route source/destination from block
endpoints and direction, checking explicit values against those endpoints.
`TaskGate._attempt_current_step()` passes the source to `DirectionArbiter.request()`.
The existing `Reservation` already stores block, direction, source, destination,
and release node; `_grant()` retains this reservation.

Before the fix, `RobotTracker.ingest_state()` selected release-node clearance,
then an allowed holding bay (no active grant, or destination only), then the
previous matching block, granted matching block, and finally the first matching
geometry. Source holding-bay telemetry immediately after a grant fell through
to an overlapping unreserved block. Original runtime fault:

```text
20:21:17.200 ARBITER_ADMIT robot=AGV_A1 block=P4_GATE_TO_RIGHT
20:21:17.368 BLOCK_FAULT robot=AGV_A1 block=P4_SIDE_TO_LEFT reason=unreserved_robot_detected_inside
20:21:17.424 TASK_RELEASED robot=AGV_A1 block=P4_GATE_TO_RIGHT goal=2108
```

The fix exposes the single retained reservation's `source_hb` and allows that
specific bay during geometric classification. It preserves block reservation
while waiting, switches to the granted block on departure, and retains existing
unresolved occupancy on a return to source. Other holding bays do not gain a
blanket exemption. No new model field, node/robot-specific production branch,
geometry expansion, forced state reset, or sleep-based robot control was added.

## RED and GREEN

The original passing-bay regression failed before production changes with
`unreserved_robot_detected_inside`. During the follow-up, the existing local fix
was preserved. The pre-fix classification branch was reconstructed **in memory**
by removing only `or hb_id == source_hb`, without replacing workspace files.
New tests then produced three expected failures: A_TO_B source LEFT lost,
B_TO_A source RIGHT lost, and actual passing-bay P4_SIDE_TO_LEFT fault.
Output is retained locally in `.runtime/source-bay-red.txt`.

Current code results:

- New regression selection: 3 tests passed (directional subtests included).
- Occupancy + passing-bay flow/config: 24 tests passed.
- Full suite: 64 tests passed, above the original baseline of 61.
- `git diff --check`: clean.
- Independent scoped review: no critical or important findings.

Coverage includes repeated stationary source telemetry, source/block geometry
overlap, both directions with source inferred by the existing config loader,
departure into the granted block, unrelated-bay/unreserved entry faults, and
retained occupancy plus timeout after a return to source. Negative test fault
messages are expected and are not runtime faults.

Commands:

```bash
.venv/bin/python -m unittest tests.test_block_occupancy.BlockOccupancyTests.test_grant_preserves_source_bay_until_departure_in_both_directions tests.test_block_occupancy.BlockOccupancyTests.test_source_grant_does_not_hide_unreserved_entry_or_unrelated_bay tests.test_passing_bay_flow.PassingBayFlowTests.test_granted_robot_still_at_source_bay_does_not_fault_overlapping_block -v
.venv/bin/python -m unittest tests.test_block_occupancy tests.test_passing_bay_flow tests.test_passing_bay_config -v
.venv/bin/python -m unittest discover -s tests -p 'test_*.py' -v
./scripts/stop_p4_passing_bay.sh
source .runtime/start_reusing_core.sh
wait
# Separate session: checks initial state, dispatches, polls, and asserts result.
.venv/bin/python .runtime/verify_source_bay_run.py
# The verifier invokes:
./scripts/t4_dispatch_passing_bay.sh
```

## Runtime scope and initial state

PID command lines and Docker ownership labels were checked before restart.
Only this workspace's Simulator, Arbiter and Fleet Adapter were restarted.
Three already-running Core containers had another workspace's ownership labels;
they were not stopped or recreated. The local runtime launcher derives from
`scripts/start_p4_passing_bay.sh`, omitting explicit Core startup and adding
`--no-deps` to Fleet Adapter recreation. A retained tool session keeps child
processes alive. Runtime helper scripts are ignored and are not committed.

At 21:15:38.420603, before dispatch, assertions confirmed A1=HB_LEFT,
B1=HB_RIGHT, jobs={}, all four blocks FREE, no faults or block queues.
The recorded arbiter log offset was 6 lines. The verifier polled status every
0.5 seconds with a 240-second overall timeout and stopped on actual completion,
not elapsed time. Any block/robot fault or BLOCKED job would fail verification.

## Actual events

Source: `.runtime/arbiter.log`, only lines after the recorded offset.

```text
2026-09-18 21:15:38,472 INFO __main__ [TRAFFIC] TASK_RELEASED robot=AGV_A1 job=1c76fe9f9e2d41be970bd91149bc5de3 block=P4_WEST_ADVANCE goal=2104
2026-09-18 21:15:39,575 INFO __main__ [TRAFFIC] TASK_RELEASED robot=AGV_B1 job=bb68a4bd10e84125af3d0e207ef934f0 block=P4_EAST_TO_SIDE goal=6137
2026-09-18 21:16:19,200 INFO __main__ [TRAFFIC] TASK_RELEASED robot=AGV_A1 job=1c76fe9f9e2d41be970bd91149bc5de3 block=P4_GATE_TO_RIGHT goal=2108
2026-09-18 21:16:46,014 INFO traffic_control.direction_arbiter [TRAFFIC] ROBOT_CLEARED_BLOCK robot=AGV_A1 block=P4_GATE_TO_RIGHT release_node=2106
2026-09-18 21:16:46,353 INFO __main__ [TRAFFIC] TASK_RELEASED robot=AGV_B1 job=bb68a4bd10e84125af3d0e207ef934f0 block=P4_SIDE_TO_LEFT goal=2101
2026-09-18 21:17:07,127 INFO __main__ [TRAFFIC] TASK_COMPLETED robot=AGV_A1 job=1c76fe9f9e2d41be970bd91149bc5de3
2026-09-18 21:17:37,158 INFO __main__ [TRAFFIC] TASK_COMPLETED robot=AGV_B1 job=bb68a4bd10e84125af3d0e207ef934f0
```

B1's second release preceded A1 completion by **20.774 seconds**.
Simulator log shows A1 arriving at 2106 and continuing toward 2107 within the
same order at 21:16:45. Polled telemetry at 21:16:46.434863 reports A1 at last
node 2106 with driving=true and B1 at last node 6137 with driving=true.

## Final state

| Robot | Job | Final node | Status | Driving |
| --- | --- | --- | --- | --- |
| AGV_A1 | 1c76fe9f9e2d41be970bd91149bc5de3 | 2108 | COMPLETE | False |
| AGV_B1 | bb68a4bd10e84125af3d0e207ef934f0 | 2101 | COMPLETE | False |

P4_WEST_ADVANCE, P4_GATE_TO_RIGHT, P4_EAST_TO_SIDE and P4_SIDE_TO_LEFT are
all FREE. Every block's occupants, reservations, and both direction waiting
queues are empty. Both domains have no active direction. Runtime BLOCK_FAULT
and ROBOT_FAULT counts are zero. Destination holding bays correctly retain
A1 in HB_RIGHT and B1 in HB_LEFT; all holding-bay reservations are empty.

All twelve requested runtime criteria passed. Evidence retained locally:

- `.runtime/source-bay-initial.json`
- `.runtime/source-bay-run-start.json`
- `.runtime/source-bay-status.jsonl`
- `.runtime/source-bay-final.json`
- `.runtime/arbiter.log` and `.runtime/simulator.log`

Original failed run: `.runtime/before-source-bay-fix-20260918-205930/`.
Earlier successful run: `.runtime/successful-run-20260918-211348/`.
Logs and runtime helper files are excluded from Git. No expansion scenarios
were implemented.
