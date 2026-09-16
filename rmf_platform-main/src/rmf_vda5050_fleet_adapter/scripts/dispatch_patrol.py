#!/usr/bin/env python3
"""RECONSTRUCTED helper that submits one robot patrol to RMF API server."""

import argparse
import json
from urllib.request import Request, urlopen


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', default='AGV_A1')
    parser.add_argument('--fleet', default='TOOL')
    parser.add_argument('--destination', default='2108')
    parser.add_argument(
        '--url', default='http://localhost:8100/tasks/robot_task'
    )
    args = parser.parse_args()
    payload = {
        'type': 'robot_task_request',
        'robot': args.robot,
        'fleet': args.fleet,
        'request': {
            'unix_millis_earliest_start_time': 0,
            'category': 'patrol',
            'priority': {'type': 'default', 'value': 0},
            'description': {'places': [args.destination], 'rounds': 1},
        },
    }
    request = Request(
        args.url,
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urlopen(request, timeout=10) as response:
        print(response.read().decode('utf-8'))


if __name__ == '__main__':
    main()
