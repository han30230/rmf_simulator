# Joint registration boundary follow-up

Date: 2026-10-04

Scope is intentionally limited to the approved two-robot A/B online path. No
solver replacement, search-budget increase, or three/four-agent expansion is
included. Earlier failure artifacts, including
`ab-holding-break-investigation` and `ab-trace-20261004-103636`, are
preserved unchanged.

## Why this change exists

The retained A/B trace showed a valid holding/bay plan being replaced after a
later legacy negotiation. Source tracing identified an additional boundary
before execution: group itineraries were written sequentially, while the
schedule node detects conflicts from an asynchronously updated mirror. That
allows a short A-new/B-old (or B-new/A-old) schedule view to open a negotiation
before the second registration is visible.

The schedule node now logs the participant registration tuple used when a new
conflict is opened:

```
conflict_version
participant
plan_id
itinerary_version
```

This instrumentation is deliberately at conflict creation, rather than inferred
later from adapter callbacks.

## Registration barrier

Core patch:

- `core-patches/0006-Expose-mirrored-itinerary-registration-version.patch`
  exposes the itinerary version already stored by `schedule::Mirror`.

Adapter patch:

- `ros2-patches/0010-Add-joint-registration-completion-boundary.patch`

The adapter now stages the group in three steps:

1. register every local participant and record
   `previous {plan, itinerary_version} -> current {plan, itinerary_version}`;
2. asynchronously wait until the fleet mirror reports the exact current pair
   for every participant;
3. only then start every `ExecutePlan`.

The barrier rejects a local registration mismatch. A timeout or validation
failure restores stationary occupancy before control is returned. Waiting can
make a solved plan historical, so start times are rechecked after mirror
convergence and the existing single stale-plan retry is used before fallback.

## Stale negotiation boundary

A negotiation is suppressed without interrupting the joint plan only when all
of the following are true:

- every referenced participant belongs to the local group;
- every referenced plan is either the recorded previous or current local plan;
- at least one reference is to the previous plan;
- the local writer still holds the expected current registration;
- after commit, the mirror also still holds the exact current
  `{plan_id, itinerary_version}` pair.

Therefore a current/current conflict is preserved, and any negotiation that
contains an external/unknown participant is preserved. Those cases keep the
existing stop -> quiesce -> stationary occupancy -> legacy negotiation path.

## Short reproduction encoded

`test_GroupPlanApplication.cpp` now contains the minimal registration race:

1. A and B start on old registrations.
2. Both writers register the new joint plans.
3. Only A's mirror is advanced to the new pair; B's mirror remains old.
4. Barrier state must be `waiting` and the start callback count remains zero.
5. B's mirror advances to its new pair.
6. Barrier becomes `ready`; only then may A and B start.

A coordinator test separately checks that mixed/previous local references are
classified stale while current/current and external references are not.

## Verification status

The code-level reproduction and instrumentation have been added to the patch
series. A fresh Docker/MQTT run was **not executed in this managed environment**:
the retained investigation already records the environment blocker
`Cannot open audit interface`. Therefore this note does not claim a new live
A/B pass.

Required next runnable-simulator checks remain, in order:

1. capture the first conflict's exact participant plan/version tuples and
   confirm whether the old run was mixed registration;
2. verify no execution callback is emitted before both registrations cross the
   barrier;
3. continue with the scheduled-departure check in the separate wait-execution
   follow-up;
4. then run the unchanged A/B bay task to 2/2 completion.

The old failures remain valid controls and have not been deleted or rewritten.
