# Reproduction commands

The executable and library paths/hashes are recorded in `environment.yaml`.

```bash
bash core_local_traffic/build_lab.sh /home/han30230/rmf-local-lab-20261003
export RMF_LOCAL_TRAFFIC_EVALUATOR=/home/han30230/rmf-local-lab-20261003/build/evaluator/local_traffic_evaluator
python3 -m unittest -v tests.test_core_evaluator tests.test_core_matrix \
  tests.test_core_budget tests.test_replan_control
python3 -m unittest discover -s tests -p 'test_*.py' -v

"$RMF_LOCAL_TRAFFIC_EVALUATOR" \
  rmf_platform-main/src/rmf_vda5050_fleet_adapter/map/p4_local_traffic.yaml \
  core_local_traffic/scenarios/opposite_3.yaml \
  > /tmp/opposite3-diag.yaml

python3 core_local_traffic/run_matrix.py \
  --binary "$RMF_LOCAL_TRAFFIC_EVALUATOR" \
  --output core_local_traffic/results/2026-10-03/matrix --repeat 1
```

The cost-only control used the same scenario with only
`cost_leeway: 100`; `solve_seconds`, `node_limit`, `extra_cost`, graph, starts,
goals and robot traits remained unchanged.

Runtime verification used Docker image `rmf-core:latest`, local host-network
MQTT at `localhost:1883`, the repository's VDA5050 simulator, the simulation
graph/config, and the adapter flag `--enable-replan-test-control`. A direct
`robot_task_request` for `AGV_A1: 2101 -> 2108` was published with reliable,
transient-local QoS. Replan was triggered with:

```bash
ros2 topic pub --once /local_traffic/replan std_msgs/msg/String '{data: AGV_A1}'
```

The RMF core containers and simulator/adapter processes started for this check
were stopped afterward.
