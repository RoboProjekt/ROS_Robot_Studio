#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
python3 /opt/studio/demo_nodes.py &
demo_pid=$!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0 &
bridge_pid=$!
cleanup() {
  kill -TERM "$demo_pid" "$bridge_pid" 2>/dev/null || true
  wait "$demo_pid" "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
# Exit if either required component fails, so the UI sees a failed container.
wait -n "$demo_pid" "$bridge_pid"
