#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import paho.mqtt.client as mqtt


parser = argparse.ArgumentParser()
parser.add_argument("output", type=Path)
parser.add_argument("--seconds", type=float, default=180.0)
args = parser.parse_args()

latest = {}
minimum = {"distance": math.inf}


def on_message(_client, _userdata, message):
    state = json.loads(message.payload)
    latest[state["serialNumber"]] = state


try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
except AttributeError:
    client = mqtt.Client()
client.on_message = on_message
client.connect("127.0.0.1", 1883)
client.subscribe("uagv/v2.0.0/inatech/+/state")
client.loop_start()
deadline = time.monotonic() + args.seconds
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open("w", encoding="utf-8") as stream:
    while time.monotonic() < deadline:
        now = time.time()
        positions = {}
        for name, state in latest.items():
            position = state["agvPosition"]
            positions[name] = {
                "x": position["x"],
                "y": position["y"],
                "last_node": state.get("lastNodeId"),
                "driving": state.get("driving"),
                "order_id": state.get("orderId"),
                "order_update_id": state.get("orderUpdateId"),
            }
        names = sorted(positions)
        for index, first in enumerate(names):
            for second in names[index + 1:]:
                a, b = positions[first], positions[second]
                distance = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                if distance < minimum["distance"]:
                    minimum = {
                        "distance": distance,
                        "unix_seconds": now,
                        "robots": [first, second],
                        "positions": [a, b],
                    }
        stream.write(json.dumps({"unix_seconds": now, "robots": positions}) + "\n")
        stream.flush()
        time.sleep(0.1)
client.loop_stop()
client.disconnect()
print(json.dumps({"minimum": minimum, "final": latest}, indent=2))
