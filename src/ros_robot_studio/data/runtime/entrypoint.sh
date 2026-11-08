#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# RuntimeManager copies only the selected model contract, without host mounts.
for attempt in $(seq 1 300); do
  if [[ -f /tmp/studio_robot.json ]]; then break; fi
  sleep 0.1
done
test -f /tmp/studio_robot.json
python3 /opt/studio/controller_nodes.py &
controller_pid=$!
python3 /opt/studio/demo_nodes.py &
demo_pid=$!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${STUDIO_BRIDGE_PORT:-9090} address:=${STUDIO_BRIDGE_ADDRESS:-0.0.0.0} &
bridge_pid=$!
cleanup() {
  kill -TERM "$demo_pid" "$bridge_pid" "$controller_pid" 2>/dev/null || true
  wait "$demo_pid" "$bridge_pid" "$controller_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
# Exit if either required component fails, so the UI sees a failed container.
wait -n "$demo_pid" "$bridge_pid" "$controller_pid"
