#!/usr/bin/env bash
set -euo pipefail

ROBOT_NAME="${1:-AGV_A1}"
DESTINATION="${2:-2108}"
FLEET_NAME="${3:-TOOL}"

docker exec -i \
  -e ROBOT_NAME="$ROBOT_NAME" \
  -e DESTINATION="$DESTINATION" \
  -e FLEET_NAME="$FLEET_NAME" \
  rmf_task_dispatcher \
  bash -lc 'source /opt/ros/jazzy/setup.bash && python3 -' <<'PY'
import json
import os
import time
import uuid

import rclpy
from rmf_task_msgs.msg import ApiRequest

rclpy.init()
node = rclpy.create_node('reconstructed_t4_dispatch')
publisher = node.create_publisher(ApiRequest, '/task_api_requests', 10)

request = ApiRequest()
request.request_id = f'reconstructed-{uuid.uuid4()}'
request.json_msg = json.dumps({
    'type': 'robot_task_request',
    'robot': os.environ['ROBOT_NAME'],
    'fleet': os.environ['FLEET_NAME'],
    'request': {
        'unix_millis_earliest_start_time': 0,
        'category': 'patrol',
        'priority': {'type': 'default', 'value': 0},
        'description': {
            'places': [os.environ['DESTINATION']],
            'rounds': 1,
        },
    },
})

deadline = time.monotonic() + 5.0
while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

publisher.publish(request)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)
print(request.request_id)
node.destroy_node()
rclpy.shutdown()
PY
