# SO-101 Follower example

Open `so101.studio.yaml` with Studio, or run from the repository:

```bash
./start.sh --demo --project examples/so101/so101.studio.yaml
```

The example references the original, pinned new_calib follower model by `model://` URI.
It contains no meshes or absolute user paths. Missing assets can be installed after
confirmation. The model has six virtual joint controls; all commands remain inside
the offline viewer. Hardware/LeRobot mappings, ROS drivers and Isaac Sim are unconfigured.

Source, revision, license, controls and verification: [SO-101 documentation](../../docs/SO101.md).
