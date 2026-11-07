# SO-101 planning profile

The bundled manifest `src/ros_robot_studio/data/plugins/so101.yaml` intentionally claims
no supported ROS distributions or executable hardware capabilities. There is no verified
SO-101 URDF, driver or calibration in this release. Selecting this profile does not change
the displayed generic robot into a SO-101. A verified description must be imported separately.

Future integration should evaluate the upstream LeRobot project and ROS 2 drivers against
the exact follower/leader hardware, firmware, calibration and licensing. No physical motion
commands are implemented in this MVP.
