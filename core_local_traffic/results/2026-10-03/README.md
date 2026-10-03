# 2026-10-03 RMF TIMEOUT and replan evidence

This directory records a fresh pinned-core build, `opposite_3` diagnostics,
a cost-only control, and two local Docker/MQTT simulation runs. No real robot
deployment was performed.

## Finding

`TIMEOUT` is not evidence of physical impossibility, and the reported 17
cost-limit events are not a configured cap. Identical 10-second runs produced
14, 15, and 15 cost-limited plan attempts. In the first run, plan calls used
9.858 s of the 10.005 s solve while rollouts used only 0.031 s.

Raising `cost_leeway` from 10 to 100 did not solve the scenario. It reduced
cost-limit events to 2 but still returned `TIMEOUT` after 10.104 s, with only
8 tables selected instead of 26. The larger ceiling made individual failed
searches more expensive and left less time to explore negotiation tables.

The structural gap is visible in candidate endpoints: rollouts included the
mainline waypoint `2105` at `(42.508, 92.871)` but never the actual side bay
`6137` at `(42.508, 89.8)`. Raising a cost ceiling cannot create an off-route
bay-yield/return candidate. The next solver milestone therefore remains the
configured bay candidate generator and complete original-goal validation from
Task 2, not another budget-only change.

A fresh baseline reproduced `VALID` for one robot (0.001 s) and two robots
(2.440 s), while the three-robot case timed out at both 10.003 s and 60.014 s
and the four-robot case timed out at 10.010 s. The 60-second control further
rules out treating this as a small timeout-budget adjustment.

## Runtime observations

The four-robot local simulation verified that a moving replan reaches the real
RMF handle, issues `startPause`, replaces the VDA5050 order, and resumes motion.
That run stayed incomplete because idle `AGV_B1` occupied goal `2108`; the
simulator logged the resulting obstacle stop loop. This is retained under
`runtime-blocked/` and is not called a pass.

The isolated one-robot run dispatched task `2101 -> 2108`, requested replan
while moving, observed replacement orders, reached `2108`, and ended with the
final RMF task-state record `status: completed` and `completed: [1]`. The topic
has two transient-local publishers and multiple cached intermediate snapshots;
verification must consume the full history and select the final record. Reading
only the first cached `underway` record gives a false failure.

## Files

- `build-verification.txt`: fresh core/evaluator build, RUNPATH/linkage and hashes.
- `environment.yaml`: host, Docker image, pinned source and artifact provenance.
- `opposite3-diag*.yaml`: three unchanged 10-second diagnostic repetitions.
- `opposite3-cost100-diag.yaml`: cost-only control.
- `single-scenario.yaml` and `single-diag.yaml`: one-robot baseline input/output.
- `diagnosis.yaml`: machine-readable conclusions and exact counters.
- `baseline-summary.yaml` and `matrix/`: fresh matrix summary and raw results.
- `runtime-blocked/`: multi-robot occupied-goal failure evidence.
- `runtime-replan-complete/`: adapter, simulator and complete task-state history.
