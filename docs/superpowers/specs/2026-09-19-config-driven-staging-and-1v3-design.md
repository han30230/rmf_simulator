# Config-driven Staging and 1v3 Passing-bay Design

Date: 2026-09-19

## Goal

Extend the passing-bay PoC with physically distinct staging slots and a
validated one-against-three scenario without encoding robot identities,
scenario sizes, or P4 node IDs in production Python.

The concrete validation scenario has one left-to-right robot and three
right-to-left robots. The implementation must remain usable for different
robot IDs, slot counts, maps, corridors, and dispatch combinations by changing
configuration and launch data.

## Scope

This change adds a separate staging-enabled P4 simulation stack. The existing
1v1, 2v1, and 2v2 maps, configuration, scripts, and regression behavior remain
available so the new physical-layout model can be compared with the existing
logical-capacity model.

The staging stack includes:

- three capacity-one staging slots on each side of the corridor;
- distinct map coordinates and graph vertices for every slot;
- managed feeder movements between slots and corridor endpoints;
- complete routes from a physical source slot to a physical destination slot;
- a 2v2 staging regression and a 1v3 runtime scenario;
- VDA5050 Simulator and Fleet Adapter registration for `AGV_B3` in the
  staging-enabled stack;
- status and log verification for movement order, final locations, resource
  cleanup, and faults.

Multi-corridor routing, multiple passing bays, automatic assignment of an
arbitrary free destination slot, and production deployment coordinates are
outside this change.

## Generalization constraints

Production Python must not contain branches for `AGV_A1`, `AGV_A2`,
`AGV_B1`, `AGV_B2`, `AGV_B3`, the 1v3 robot count, or P4 node IDs. It must not
infer policy from robot-name prefixes.

The following are configuration data:

- holding-bay and slot IDs;
- graph node IDs and surveyed coordinates;
- slot capacity;
- feeder and corridor block endpoints;
- direction domains;
- allowed route steps;
- release nodes;
- policies that require opposite route work to clear;
- scenario robot selection, initial positions, and requested destinations.

The Python runtime continues to operate on `HoldingBay`, `CorridorBlock`,
`RouteStep`, `RouteIntent`, and direction-domain semantics. A field deployment
replaces simulation graph coordinates and YAML route definitions without
changing admission code.

Scenario scripts may name robots and destinations because they are launch
fixtures, not production admission policy.

## Physical graph

The staging map has three slots on each side:

```text
LEFT_SLOT_1  --\
LEFT_SLOT_2  ---- LEFT_ENTRY -- 2102 -- 2103 -- 2104 -- 2105 -- 2106 -- 2107 -- RIGHT_ENTRY ---- RIGHT_SLOT_1
LEFT_SLOT_3  --/                                      |
                                                    SIDE_BAY
                                                                                               \--- RIGHT_SLOT_2
                                                                                                \-- RIGHT_SLOT_3
```

The final graph will use non-overlapping side spurs rather than placing
multiple vertices at one coordinate. Each spur connects only to its local
corridor entry. Slot geometry circles do not overlap each other, the corridor
block geometry, or the entry holding-bay geometry.

Both sides have three slots. This is required even though the 2v2 start state
uses only two slots per side:

- the 1v3 final state needs three distinct left-side destinations;
- the 2v2 run can temporarily have an east-side waiter plus two arriving
  robots, so a spare east slot prevents endpoint deadlock;
- symmetric capacity makes the map reusable for reversed scenarios.

The simulation coordinates demonstrate topology and separation only. Field
coordinates must be replaced with surveyed values that include vehicle
footprint, stopping accuracy, and the required safety margin.

## Resource model

Every physical slot is a separate `HoldingBay` with capacity one. The shared
corridor entry remains a capacity-one holding bay.

Each slot spur is represented by a managed feeder block. Feeder blocks on the
same side share a direction domain so only compatible movement can use the
merge area. The entry holding-bay reservation makes movement across the merge
atomic: a robot cannot leave a slot until the entry is reserved, and a robot
cannot leave the corridor entry for an occupied destination slot.

The west approach and passing-event domains retain their current roles:

- the west-approach domain pipelines same-direction robots toward the gate;
- the passing-event domain protects the main conflict area and side-bay merge;
- configured `release_node` telemetry clears an active crossing only after the
  robot passes the physical conflict boundary;
- `requires_opposite_routes_cleared` keeps a side-bay robot stopped while any
  opposite job still has current or future work in that conflict domain.

No slot is treated as safe merely because it is a holding bay. It is safe only
when it is the configured source or destination of the robot's retained grant,
or when the robot is stationary there without an active grant under the
existing tracker rules.

## Route representation

The current route model already supports an arbitrary sequence of managed
steps. Staging routes therefore use explicit YAML steps rather than a
scenario-specific scheduler:

1. source slot to local corridor entry;
2. existing safe corridor steps;
3. remote corridor entry to the assigned destination slot.

Allowed source/destination combinations are explicit route intents. This is
site routing policy, equivalent to allowed lanes in an RMF graph. It avoids a
new route-composition engine while keeping runtime behavior independent of
robot IDs.

Routes used by the first validation assign destinations that are physically
available when needed:

- the first right-to-left robot vacates a right slot before the
  left-to-right robot needs that slot;
- the second left-to-right robot in 2v2 uses the spare third right slot;
- right-to-left robots use distinct left slots vacated by departing robots or
  initially empty slots.

If a destination remains occupied, its atomic holding-bay reservation denies
the feeder grant and the robot waits at the corridor entry. Configuration and
route ordering must prevent cyclic slot swaps. The tests include this resource
condition so a future map change cannot silently introduce endpoint deadlock.

## Scenario behavior

### Staging 2v2

The staging 2v2 scenario demonstrates that four robots start and finish at
distinct coordinates. It preserves the safe order established by the current
2v2 scenario:

1. B1 enters the side bay.
2. A1 crosses and clears its release node.
3. A2 advances and crosses.
4. B1 remains in the side bay until A2 clears its release node.
5. B1 leaves for its assigned left slot.
6. B2 uses a direct route after opposite passing-event work is clear.
7. all four robots finish in distinct destination slots.

### Staging 1v3

The one-against-three scenario starts A1 in a left slot and B1/B2/B3 in three
right slots:

1. A1 advances from its slot toward the west gate.
2. B1 leaves its slot and enters the side bay.
3. B2 and B3 remain in their right slots.
4. A1 crosses without stopping at the release node.
5. B1 leaves the side bay only after A1 clears the conflict boundary.
6. A1 exits the corridor into the right slot vacated by B1.
7. B2 and B3 are admitted in FIFO order and use the direct route while the
   direction remains right-to-left.
8. B1, B2, and B3 finish in three different left slots.

The expected order follows submission time and resource availability. Python
does not assign priority based on the robot suffix.

## Failure behavior

The existing fail-closed behavior remains mandatory:

- missing or stale telemetry inside a block faults the relevant block;
- a robot outside its configured source, destination, or granted block is an
  unreserved intrusion;
- a missing release-node report keeps opposite traffic stopped;
- an occupied destination slot keeps the requesting feeder step waiting;
- an upstream HTTP rejection releases reservations and blocks the job;
- an uncertain upstream response retains the grant and does not submit a
  duplicate task automatically.

The new staging geometry must not be widened to hide classification errors.
Geometry represents measured physical occupancy; transition safety comes from
the retained grant and its configured endpoints.

## Configuration and runtime files

The implementation will add staging-specific files alongside the current P4
files:

- a navigation graph with six slot nodes and feeder lanes;
- a Simulator scenario with distinct robot positions and `AGV_B3`;
- a Fleet Adapter configuration registering the staging robots;
- an Arbiter configuration containing slot holding bays, feeder blocks, and
  complete routes;
- start and dispatch scripts for staging 2v2 and staging 1v3;
- documentation explaining how to replace simulation coordinates and route
  policy for a field deployment.

Runtime helpers and generated `.runtime` files remain ignored. User recordings
are never added to commits.

## Test strategy

Implementation follows RED-GREEN TDD.

Configuration tests first fail unless:

- all six slot vertices have distinct coordinates;
- slot geometry does not overlap other slots or managed corridor geometry;
- each slot has capacity one;
- every feeder edge exists in both the graph and Arbiter configuration;
- the Simulator initial positions match distinct slot coordinates;
- the Fleet Adapter contains all robots selected by each runtime script;
- staging scripts use repository-relative paths;
- production Python contains no scenario robot or P4-node branches.

Flow tests exercise real `CorridorRegistry`, `DirectionArbiter`, `RobotTracker`,
and `TaskGate` objects. They verify:

- feeder admission reserves the entry atomically;
- a second robot at the same-side merge waits;
- a destination slot occupant prevents admission;
- staging 2v2 keeps B1 in the side bay until A2 clears;
- staging 1v3 releases B1 after A1 clears, then pipelines B2 and B3 in FIFO
  order;
- final jobs are complete at six physically distinct slot nodes;
- every block has no fault, occupant, reservation, or waiter.

The full existing test suite must remain green and contain at least the current
69 tests plus the new staging regressions.

## Runtime verification

After all tests pass, only this repository's runtime is restarted. The
staging-enabled VDA5050 Visualizer run is condition-polled rather than judged
after a fixed sleep.

For both staging 2v2 and 1v3, verification records:

- initial distinct positions and empty jobs;
- each `TASK_RELEASED`, `ROBOT_CLEARED_BLOCK`, `ROUTE_SWITCH`, and
  `TASK_COMPLETED` event in order;
- the side-bay robot's departure relative to every required opposite
  release-node event;
- final robot nodes and `driving=false`;
- all jobs `COMPLETE`;
- all blocks `FREE`;
- empty block occupants, reservations, and waiting queues;
- empty holding-bay reservations;
- zero `BLOCK_FAULT` and `ROBOT_FAULT` events for the run window.

The change is committed and pushed only after unit tests and both actual
runtime scenarios pass.

## Field deployment boundary

This change produces a reusable admission model, not certified field geometry.
Before real deployment, the site integrator must replace coordinates and
validate vehicle dimensions, braking distance, localization tolerance,
communications timeout, PLC or safety-controller interfaces, emergency-stop
behavior, and manual recovery procedures. Those values remain external to the
generic admission algorithm.
