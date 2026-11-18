"""Step a local MuJoCo model and expose its measured joint state in URDF units."""

import math


class LocalMujoco:
    def __init__(self, model, scene, mapping, initial):
        try:
            import mujoco as mj
        except ImportError as exc:
            raise ValueError(
                "Install the Studio MuJoCo extra in .venv: pip install '.[mujoco]'"
            ) from exc
        self.mj = mj
        names = dict(model.default_configuration())
        if not names or (not scene and not model.source_path):
            raise ValueError(
                "Load a robot with movable joints and a URDF, or select a MuJoCo scene."
            )
        mapping = {n: mapping.get(n, n) for n in names}
        if len(set(mapping.values())) != len(mapping):
            raise ValueError("MuJoCo joint mapping must be one-to-one.")
        spec = mj.MjSpec.from_file(str(scene or model.source_path))
        if not scene:
            for remote in mapping.values():
                actuator = spec.add_actuator(
                    name="studio_" + remote, target=remote, trntype=mj.mjtTrn.mjTRN_JOINT
                )
                actuator.set_to_position(kp=15.0, kv=1.0)
        try:
            self.model = spec.compile()
        except ValueError as exc:
            if scene or "mass and inertia of moving bodies" not in str(exc):
                raise
            # The Studio kinematic demos omit inertials. Apply defaults only
            # when MuJoCo cannot compile the original URDF as supplied.
            spec.compiler.boundmass = 0.1
            spec.compiler.boundinertia = 0.001
            self.model = spec.compile()
        self.data = mj.MjData(self.model)
        self.joints = {}
        self.actuators = {}
        studio_joints = {joint.name: joint for joint in model.joints}
        for name, remote in mapping.items():
            self.add_joint(studio_joints[name], remote, model.effective_joint_bounds[name])
        if not scene:
            for name, value in initial.items():
                qpos, _, lower, upper = self.joints[name]
                self.data.qpos[qpos] = max(lower, min(upper, value))
        for name in self.actuators:
            self.target(name, float(self.data.qpos[self.joints[name][0]]))
        mj.mj_forward(self.model, self.data)

    def add_joint(self, joint, remote, bounds):
        mj, m = self.mj, self.model
        name = joint.name
        index = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, remote)
        expected = mj.mjtJoint.mjJNT_SLIDE if joint.kind == "prismatic" else mj.mjtJoint.mjJNT_HINGE
        if index < 0 or m.jnt_type[index] != expected:
            raise ValueError(f"MuJoCo joint {remote} is missing or has a different joint type.")
        lower, upper = bounds
        if m.jnt_limited[index]:
            lower, upper = max(lower, m.jnt_range[index][0]), min(upper, m.jnt_range[index][1])
        if lower > upper:
            raise ValueError(f"{remote} has no shared MuJoCo and URDF travel.")
        self.joints[name] = (int(m.jnt_qposadr[index]), int(m.jnt_dofadr[index]), lower, upper)
        matching = [
            i
            for i in range(m.nu)
            if m.actuator_trntype[i] == mj.mjtTrn.mjTRN_JOINT and m.actuator_trnid[i][0] == index
        ]
        if len(matching) == 1:
            i = matching[0]
            kp = m.actuator_gainprm[i][0]
            if (
                kp > 0
                and m.actuator_gaintype[i] == mj.mjtGain.mjGAIN_FIXED
                and m.actuator_biastype[i] == mj.mjtBias.mjBIAS_AFFINE
                and m.actuator_dyntype[i] == mj.mjtDyn.mjDYN_NONE
                and math.isclose(m.actuator_biasprm[i][1], -kp)
                and math.isclose(m.actuator_gear[i][0], 1.0)
            ):
                self.actuators[name] = i

    def target(self, name, position):
        i = self.actuators.get(name)
        if i is None:
            raise ValueError(f"No position actuator is configured for {name}.")
        _, _, lower, upper = self.joints[name]
        if self.model.actuator_ctrllimited[i]:
            lo, hi = self.model.actuator_ctrlrange[i]
            lower, upper = max(lower, lo), min(upper, hi)
        if not math.isfinite(position) or lower > upper:
            raise ValueError(f"Invalid position target for {name}.")
        self.data.ctrl[i] = max(lower, min(upper, position))

    def step(self, seconds=0.03):
        nstep = min(100, max(1, round(seconds / self.model.opt.timestep)))
        self.mj.mj_step(self.model, self.data, nstep=nstep)
        positions, velocities = {}, {}
        clipped = False
        for name, (qpos, qvel, lower, upper) in self.joints.items():
            value = float(self.data.qpos[qpos])
            if not math.isfinite(value):
                raise ValueError(f"Non-finite MuJoCo feedback for {name}.")
            bounded = max(lower, min(upper, value))
            if bounded != value:
                self.data.qpos[qpos] = bounded
                self.data.qvel[qvel] = 0
                clipped = True
            positions[name] = bounded
            velocities[name] = float(self.data.qvel[qvel])
        if clipped:
            self.mj.mj_forward(self.model, self.data)
        return positions, velocities
