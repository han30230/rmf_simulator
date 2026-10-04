# A/B holding-plan interruption investigation

Date: 2026-10-04

Scope: two opposing tasks (`AGV_A1 -> 2108`, `AGV_B1 -> 2101`) only. This
note intentionally does not propose a solver, budget, or traffic-policy change.

## Confirmed from the retained Docker/MQTT evidence

The retained 10-second-budget run contains a two-agent interval before either
additional robot receives an order. Timestamps below are monitor Unix seconds;
wall-clock equivalents come from the simulator logs.

| Time | Robot | Event |
|---:|---|---|
| 1791070358.091 | A1 | VDA `order_3_5bd6ab22`, path `2101 -> 2102`, starts |
| 1791070358.091 | B1 | VDA `order_1_d7f4978e`, path `2108 -> 2107`, starts |
| 1791070365.913 | A1 | driving becomes false at `(14.76, 92.87)` |
| 1791070365.913 | B1 | driving becomes false at `(66.90, 92.87)` |
| 2026-10-03 23:32:45 | A1/B1 | both simulator logs receive `startPause` |
| 1791070366.615 | A1 | replacement `order_6_4e12f523` begins |
| 1791070366.615 | B1 | replacement `order_3_3d1edbcc` begins |
| 1791070393.390 | B2 | first additional-robot order begins |

Therefore the first interruption of the two-agent execution happened about
7.8 seconds after the paired VDA orders began and about 27.5 seconds before the
first C/D order. It cannot be attributed to the later C/D task injection.

The same retained run also proves that a bay command can cross the complete
RMF-to-VDA boundary: A1 received `order_12_0405823a` with nodes
`['2105', '6137']` at 2026-10-03 23:33:29. It moved about 0.12 m toward the bay,
then received `startPause` at 23:33:30. A replacement main-line order
`order_14_83a186d3`, nodes `['2105', '2106', '2107', '2108']`, arrived at
23:33:43. This later event occurred after B2 was active, so it is supporting
evidence for plan replacement, not the clean two-agent reproduction requested
for the final diagnosis.

## Relevant call path

After `GroupPlanApplication::apply()` reports success,
`GoToPlace::Active::_try_local_planning()` calls
`LocalJointPlanningCoordinator::complete_local_cycle()`. That calls
`finish_local_cycle()`, which immediately changes every participant owner from
`local_applying` to `ordinary`. A subsequent `GoToPlace::Active::_respond()` is
therefore permitted to call the legacy `services::Negotiate::path()` approval
callback, whose `_execute_plan()` replaces the current `ExecutePlan` and emits
new navigation commands.

This is a demonstrated *possible* replacement path. The retained adapter log
summary records failed negotiations after successful local starts, but it does
not contain participant generation, RMF plan ID, or VDA order ID on the same
timestamped records. Consequently it is not yet sufficient to prove that this
exact callback caused the first 23:32:45 pause.

## Destination occupancy

The original goals are distinct endpoints: `2101=(6.833,92.871)` and
`2108=(73.7274,92.8248)`, approximately 66.894 m apart. Each is the other
robot's vacated start, and neither lies between the bay branch (`2105`) and the
other endpoint. Terminal occupancy at these two endpoints therefore does not
by itself make the A/B swap physically impossible. This differs from the
earlier fixed-2108 case, where a stationary participant already occupied the
requested goal.

## Missing evidence and rerun status

An A1/B1-only simulator fixture is stored at
`../../fixtures/simulator-ab.yaml`. Starting/stopping Docker workloads was
blocked in the current managed execution environment: read-only `docker ps`
works through the docker group, while Docker state-changing calls are rejected
before reaching Compose (`Cannot open audit interface`). No fresh two-agent
run was therefore produced in this continuation.

No code fix has been applied. The next valid run must capture, on one timeline:

- local cycle, participant, intent generation, and assigned RMF plan ID;
- `complete_local_cycle` and every `_respond` ownership transition/reason;
- `follow_new_path` plan ID and VDA command/order ID;
- simulator order receipt, pause reason, pose, and driving state.

Only after those records identify the first causal callback should the smallest
change be made and the identical A/B run repeated.

## Verification retained in this continuation

- `test_LocalJointPlanningCoordinator_latest`: 47 assertions / 13 cases pass.
- `test_GroupPlanApplication`: 20 assertions / 4 cases pass.
- No Fleet Adapter, ROS adapter, core solver, budget, or traffic policy source
  was changed.
