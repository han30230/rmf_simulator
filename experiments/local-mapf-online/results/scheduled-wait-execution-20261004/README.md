# Scheduled wait execution follow-up

Date: 2026-10-04

This is the second approved narrow fix. It does not change the local solver,
search budgets, group-size policy, topology policy, or the previously retained
failure results.

Patch:

- `ros2-patches/0011-Honor-scheduled-departure-in-EasyFullControl.patch`

## Source trace

The execution path was traced through:

`ExecutePlan -> MoveRobot -> RobotCommandHandle::follow_new_path -> EasyFullControl`.

EasyFullControl already carried an explicit `planned_wait_time` when waypoint
collapsing detected a planned wait. Before this follow-up that value was only
added to the arrival estimate:

```
arrival_estimator(estimated_motion_time + planned_wait_time)
```

The physical command queue itself immediately invoked the next
`NavigationRequest` when the previous command reported completion. Therefore
schedule timing could describe a safe wait while the robot command stream left
early.

## Execution change

Each normal navigation command now carries its source waypoint's planned
departure time.

`ProgressTracker` does not call the user `NavigationRequest` before that
time. It polls `RobotContext::now()` asynchronously, so the fleet worker is
not blocked and simulated/ROS time remains the authority.

This also covers non-collapsed wait waypoints: if a no-op wait segment finishes
early, the following motion command remains gated by the wait-end waypoint time.

When rotation-command collapsing folds an explicit wait into one navigation
command, an `earliest_finish` is also recorded. If the robot reports that
navigation complete before the folded wait ends, command completion is delayed
until that wait end. This prevents a final collapsed wait from disappearing
when there is no subsequent motion command to gate.

Existing `CommandExecution::make_hold` paths used by localization/lift handling
are not converted or duplicated.

## Cancellation boundary

A command cancelled while its planned-start timer is pending has not yet been
sent to the robot. Cancellation therefore:

1. invalidates the RMF activity;
2. cancels the planned-start timer;
3. does **not** call the user robot-stop callback a second time.

Likewise, if the robot has already reported navigation complete and only the
planned-finish wait remains, cancelling that timer does not emit another
robot-level stop.

A command that has actually been sent and is still moving keeps the original
stop behavior.

## Verification status

The source-level omission and the correction are recorded in the patch series.
A fresh simulator run could not be launched in this managed environment because
the retained Docker investigation is blocked by `Cannot open audit interface`.
No new live success is claimed here.

The required runtime checks remain in the approved order:

1. registration-boundary short reproduction / first-conflict tuple capture;
2. construct or observe a command with a future planned departure and verify
   that no `NavigationRequest` and no robot displacement occurs before it;
3. run the same A/B case and require bay entry, pass, exit, and task 2/2
   completion;
4. delay one peer and require either the planned safe wait to be honored or a
   stopped joint replan, never early entry.

The previous failed traces are retained as before/after controls.
