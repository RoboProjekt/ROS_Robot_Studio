#!/bin/bash
set -Ee
trap 'studio_failure_code=$?; echo "STUDIO_RUNTIME_PROCESS_FAILED exit=$studio_failure_code line=$LINENO" >&2' ERR
source "/opt/ros/${ROS_DISTRO}/setup.bash"
for attempt in $(seq 1 300); do
  if [[ -f /tmp/studio_robot.json ]]; then break; fi
  sleep 0.1
done
test -f /tmp/studio_robot.json
profile=$(python3 -c 'import json; print(json.load(open("/tmp/studio_robot.json"))["profile"])')
pids=()
# Read-only connections have no action gateway, controller or demo publisher.
if [[ "$profile" != read_only ]]; then
  python3 /opt/studio/controller_nodes.py &
  pids+=($!)
fi
if [[ "$profile" == managed_simulation ]]; then
  python3 /opt/studio/demo_nodes.py &
  pids+=($!)
fi
ros2 launch /opt/studio/studio_bridge.launch.py &
pids+=($!)
cleanup() {
  kill -TERM "${pids[@]}" 2>/dev/null || true
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
wait -n "${pids[@]}"
