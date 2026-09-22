# Connected Corridor Chain Design

## Purpose

The target factory layout is one connected, bidirectional, single-lane route
with side bays between corridor segments. It is not a collection of physically
separate corridors. Up to four robots may use the chain. Robots should wait at
an endpoint or inside a side bay during normal operation so that a planned stop
does not obstruct the main lane.

The first proof of concept has three consecutive corridor segments and two
intermediate side bays:

```text
LEFT_END
    |
    C1
    |
JUNCTION_1 -- SIDE_1
    |
    C2
    |
JUNCTION_2 -- SIDE_2
    |
    C3
    |
RIGHT_END
```

`LEFT_END`, `RIGHT_END`, `SIDE_1`, and `SIDE_2` are safe stops. The junctions
are graph nodes used to enter, leave, or pass a side bay; they are not normal
waiting locations. Emergency stops remain possible anywhere, but they put the
affected resources into a fail-closed state.

## Design Goals

- Let a robot drive directly across multiple clear segments without entering
  every side bay.
- Use a side bay only when it is needed for a conflict-free passing plan.
- Keep planned waiting off the single-lane main route.
- Pipeline robots moving in the same direction when capacity permits.
- Allow opposite directions concurrently on non-overlapping segments.
- Prevent head-on entry, destination overbooking, cyclic waits, and starvation.
- Treat four robots as a configured admission limit rather than a robot-count
  branch in production code.
- Express topology, capacity, geometry, and release policy in configuration.
- Preserve fail-closed behavior when telemetry or occupancy is uncertain.

## Non-goals

- This proof of concept does not model more than one connected chain or a
  general graph with forks between multiple destination areas.
- It does not replace the robot safety controller, safety PLC, obstacle
  detection, or emergency-stop system.
- It does not infer field dimensions from the simulation map. Production
  geometry must use surveyed coordinates, robot dimensions, braking distance,
  localization error, and an explicit safety margin.

## Configured Topology

The registry will gain physical holding-bay slots, logical safe-stop groups,
and an ordered corridor chain. A group may contain several distinct endpoint
slots, while a side-bay group normally contains one slot. Names and node
identifiers below are symbolic examples; production Python must not depend on
them.

```yaml
holding_bays:
  LEFT_SLOT_1:
    type: endpoint_slot
    node_id: LEFT_SLOT_NODE_1
    capacity: 1
  LEFT_SLOT_2:
    type: endpoint_slot
    node_id: LEFT_SLOT_NODE_2
    capacity: 1
  SIDE_1:
    type: side_bay
    node_id: SIDE_BAY_NODE_1
    access_node: MAIN_JUNCTION_NODE_1
    capacity: 1
  SIDE_2:
    type: side_bay
    node_id: SIDE_BAY_NODE_2
    access_node: MAIN_JUNCTION_NODE_2
    capacity: 1
  RIGHT_SLOT_1:
    type: endpoint_slot
    node_id: RIGHT_SLOT_NODE_1
    capacity: 1
  RIGHT_SLOT_2:
    type: endpoint_slot
    node_id: RIGHT_SLOT_NODE_2
    capacity: 1

safe_stop_groups:
  LEFT_END:
    members: [LEFT_SLOT_1, LEFT_SLOT_2]
  SIDE_1_STOP:
    members: [SIDE_1]
  SIDE_2_STOP:
    members: [SIDE_2]
  RIGHT_END:
    members: [RIGHT_SLOT_1, RIGHT_SLOT_2]

corridor_chains:
  - id: FAB_MAIN_LINE
    blocks: [C1, C2, C3]
    safe_stops: [LEFT_END, SIDE_1_STOP, SIDE_2_STOP, RIGHT_END]
    max_active_robots: 4
```

The abbreviated example shows two physical slots per endpoint; the configured
member list may contain up to the number supported by the measured endpoint
area. Each member has its own graph node and geometry, so robots never share a
single coordinate merely because the logical endpoint has capacity greater
than one.

The ordered `safe_stops` list has exactly one more element than `blocks`.
Block `C1` connects `LEFT_END` and `SIDE_1`, `C2` connects `SIDE_1` and
`SIDE_2`, and `C3` connects `SIDE_2` and `RIGHT_END`. A side-bay block path
includes its access spur for entry and exit. A direct authority across adjacent
blocks follows the main graph through the junction and does not visit the side
bay node.

Configuration validation rejects duplicate resources, disconnected block
orders, endpoint mismatches, missing access nodes, non-positive capacities,
and ambiguous membership of a block or safe stop in the first proof-of-concept
chain.

## Movement Authority

The current single-block reservation is extended with a movement authority
that owns an ordered interval of blocks and one destination safe-stop slot.

```text
MovementAuthority
  authority_id
  robot_id
  chain_id
  source_safe_stop
  destination_safe_stop
  source_slot
  destination_slot
  ordered_blocks
  direction
  request_time
  block_release_conditions
  state
```

An authority is the permission to move from one safe-stop group to a concrete
slot in another group without a planned stop on the main lane. All required
blocks and the selected destination slot are reserved atomically under the
arbiter lock. If any resource is unavailable, the request obtains no partial
reservation and remains in its source slot.

The authority retains the configured direction for every reserved block. A
block may be shared only by compatible same-direction authorities and only up
to its configured capacity. Opposite-direction authorities cannot reserve an
overlapping block interval.

The destination reservation remains held until telemetry confirms arrival.
Blocks behind the robot are released in configured order when their release
condition is observed. Releasing a passed block does not release the destination
safe-stop reservation.

## Rolling Reservation and Destination Selection

When Task Gate receives a final destination request, the chain planner finds
the ordered blocks between the robot's current safe-stop group and the final
safe-stop group. The scheduler then chooses the farthest safe-stop group that
can be reached with an atomic, conflict-free authority and reserves one
concrete member slot.

If every required resource is available, the destination is the final endpoint
and RMF receives one continuous task across all three blocks. Intermediate
side bays are passed on the main lane.

If the full interval conflicts with an opposite movement, the scheduler may
choose the last available side bay before the conflict. The side bay is used
only when its slot is reserved and the corresponding path is safe. After
telemetry confirms arrival inside the bay, the scheduler requests the next
authority toward the final destination.

An authority already issued to a moving robot is not revoked for a later
request. Later conflicting robots wait at their current safe stops. This avoids
mid-route diversion based on timing and gives each RMF task a stable goal.

Before a robot reaches its granted destination, Task Gate may calculate a
continuation. It submits a continuous next leg only after the continuation and
its destination are safely reserved. If no continuation can be reserved, the
robot completes the current leg inside its already reserved safe stop.

## Admission, Batching, and Fairness

Existing authorities have priority because their robots may already be moving.
Requests that do not overlap may run concurrently, including opposite
directions on C1 and C3. Requests with overlapping block intervals use the
following policy:

1. Preserve every active movement authority.
2. Admit compatible same-direction followers while the direction batch is
   open and block/destination capacity permits.
3. Close a direction batch when an opposite-direction waiter appears.
4. Let already admitted robots reach safe stops and release the overlap.
5. Select the oldest waiting direction, with accumulated wait time breaking
   ties and preventing starvation.
6. Within that direction, select requests by request time.

No robot identifier, endpoint number, or fixed preferred direction participates
in this decision. The configured `max_active_robots` limits admission into the
chain but does not change ordering rules.

A robot entering a side bay does not make the overlapping opposite route safe
until telemetry classifies it inside that bay and outside the main-lane conflict
geometry. Only then may the opposite authority be released.

## Task and Telemetry Flow

1. Task Gate receives the robot's requested final destination slot.
2. Robot Tracker supplies the current node, safe stop, block occupancy, and
   telemetry health.
3. The chain planner maps the physical slots to their groups and derives the
   ordered route interval without enumerating
   every robot/start/goal combination in Python.
4. The scheduler selects the farthest safe reachable destination.
5. The arbiter atomically reserves the ordered blocks and destination slot.
6. Task Gate submits one RMF task for that safe destination.
7. Robot Tracker follows VDA5050 node, edge, driving, and position telemetry.
8. Confirmed release conditions free blocks behind the robot in order.
9. Arrival completes the authority and releases its destination reservation
   after occupancy has been established.
10. If the final destination has not been reached, Task Gate repeats the
    process from the current safe stop.

The tracker prefers blocks reserved by the robot when geometries overlap. The
configured source and destination safe stops remain valid transition areas for
their authority. An unrelated holding bay must not hide an unreserved block
entry.

## Failure Handling

- **Telemetry timeout while an authority is active:** mark every unreleased
  block held by that authority fail-closed. Do not infer that the robot exited.
- **Ambiguous position or destination occupancy:** deny new authorities whose
  safety depends on that resource.
- **Explicit upstream task rejection:** atomically release the new authority
  and destination reservation, then report or retry according to existing Task
  Gate policy.
- **Uncertain upstream response:** retain the authority and block conflicting
  traffic until the task state is reconciled.
- **Emergency or obstacle stop on the main lane:** keep the current and
  unreleased forward resources closed. Operational recovery is required before
  normal admission resumes.
- **Process restart with uncertain occupancy:** reconstruct only facts
  confirmed by telemetry. Unresolved blocks remain closed and require
  reconciliation or an explicit safe reset.
- **Side-bay entry not confirmed:** do not release the opposite direction.

## First Simulation Map

The connected proof-of-concept map will have one horizontal main route, three
single-lane segments, two distinct side spurs, and endpoint slot groups. Robot
simulator start and destination positions are physically distinct when more
than one robot uses the same side. Staging slots connect to the endpoint area
without changing the three managed corridor segments.

The existing disconnected P4/P5 scenario remains a regression test for
independent direction domains. It is not presented as the model of the target
factory line.

## Verification

Implementation follows test-driven development.

### Unit and State-machine Tests

- Parse and validate a three-block connected chain.
- Derive both directions between every pair of safe stops.
- Reserve an ordered block interval and destination slot atomically.
- Leave no partial reservation after a failed request.
- Select the final endpoint on an entirely clear chain.
- Select a side bay only when a farther interval conflicts.
- Release passed blocks in order while retaining the destination reservation.
- Permit opposite directions on disjoint intervals.
- Reject opposite directions on overlapping intervals.
- Close a same-direction batch after an opposite waiter arrives.
- Age waiting directions so neither side starves.
- Fail closed all unreleased blocks after telemetry loss.
- Preserve the source/destination transition classification at overlapping
  geometry without masking an unrelated intrusion.

### Deterministic Four-robot Scenarios

- One against three from initial state.
- Two against two from initial state.
- B2/B3 or their symmetric peers inserted while robots are moving.
- All four starting from distinct physical staging slots.
- Clear-chain trailing robots travel directly without unnecessary side-bay
  visits.
- Opposite robots run concurrently only when their reserved intervals are
  disjoint.
- Every normal planned stop occurs at an endpoint or inside a side bay.

### Fault Injection

- Telemetry timeout in each of C1, C2, and C3.
- Missing side-bay arrival telemetry.
- Upstream rejection and uncertain upstream response.
- Stale occupancy at startup.

### Actual Runtime Acceptance

For each scenario, record the log offset before dispatch and verify only the
new run. Success requires:

- every expected task and movement authority reaches `COMPLETE`;
- final robot nodes match their requested destinations;
- no `BLOCK_FAULT` or `ROBOT_FAULT` occurs in a normal run;
- no opposite authorities overlap the same corridor block;
- no robot makes a planned stop on a junction or main-lane segment;
- side-bay entry occurs only when selected as a conflict refuge;
- all blocks finish `FREE`;
- block occupants, reservations, and waiting queues are empty;
- destination safe-stop occupancy matches the final robot positions; and
- the VDA5050 visualizer shows collision-free motion for the complete run.

The focused tests run before the full repository test suite. Actual simulation
starts only after all tests pass. A change is complete only after both connected
1v3 and connected 2v2 runtime acceptance pass.

## Implementation Boundaries

- Robot and node names belong in maps, fleet configuration, scenario data, and
  traffic-policy YAML, not in production Python conditionals.
- Timing sleeps do not make admission decisions; state polling and telemetry do.
- Geometry represents physical occupancy and is not enlarged to hide state
  transition errors.
- RMF remains responsible for path execution. The arbiter controls when and how
  far a robot may receive a task through the managed single-lane chain.
- Field deployment requires measured geometry and safety-system integration in
  addition to successful simulation.
