# Local traffic core experiment implementation plan

> Use superpowers:executing-plans; implement in the authorized feature branch.

Goal: evaluate real RMF centralized planning, provide reproducible core overlays and actual adapter replan controls; only enable live group application after solver and runtime evidence.
Architecture: pin standalone rmf_utils/rmf_traffic/rmf_ros2, keep core modifications as version-checked patches, add a YAML-driven C++ evaluator and opt-in ROS replan test subscription. Preserve the arbiter baseline.
Tech stack: C++17, Open-RMF Jazzy, Python unittest, YAML.
Spec: ../specs/2026-10-02-local-traffic.md

Global constraints: no fabricated runtime success; no robot-name hardcoding in solver; default feature off; retain original task goals; no expired reservation as physical clearance.

Tasks:
1. Write failing configuration tests, add physically separated bay/parking graph and simulation config; run unittest and commit.
2. Compile real rmf_traffic; add C++ evaluator with start/tail collision validation and deadline. Test one robot, opposite directions with/without bay, invalid overlapping starts and idle blockers. Record results before choosing any live group hook.
3. Add tested version-checked core patches for configurable search limits/cancellation and diagnostics. Add actual opt-in adapter replan trigger and command logging, with behavioral unit tests. Provide overlay preparation/build scripts.
4. Run all available tests, attempted actual runtime preflight, document solver findings and unmet runtime acceptance criteria. Independent review; fix important findings; push branch.

Shared interfaces: Task 1 graph is consumed by Task 2 and runtime setup. Task 2 output schema is consumed by scenario checks. Task 3 patch manifest is consumed by preparation/build scripts.
Ruling: source-only/public snapshot lacks the original rmf-core checkout and running ROS environment. Pin upstream only for a separate laboratory overlay, not as an upgrade to the user's installed deployment.
Review focus: end occupancy; cancellation; stale core base; opt-in test controls; unsupported runtime must fail clearly rather than report passed.

## Recorded outcome

Laboratory tasks completed; 49 tests pass with real traffic binary enabled. Exact-version patch application checked. 2-agent bay crossing passes; 3/4-agent static joint solves timeout at 10 seconds; 3-agent also times out at 60 seconds. ROS fleet-adapter build and all dynamic runtime acceptance remain blocked by missing ROS/colcon/Docker. Shared live coordinator is not implemented because this backend did not satisfy group planning gate. Partial reviewer findings corrected; reviewer usage limit prevented final review completion. Final report must state this is a milestone, not the full requested implementation.
