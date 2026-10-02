#!/usr/bin/env python3
"""Inventory required runtime commands; missing tools are BLOCKED, not test pass."""
import shutil
import sys
import yaml
required=('ros2','colcon','docker')
state={name:shutil.which(name) for name in required}
print(yaml.safe_dump({'commands':state,'status':'READY_FOR_SETUP' if all(state.values()) else 'BLOCKED'},sort_keys=False))
sys.exit(0 if all(state.values()) else 2)
