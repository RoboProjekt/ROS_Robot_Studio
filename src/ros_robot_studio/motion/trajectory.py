"""Synchronized cubic joint-space preview with analytic velocity/acceleration checks."""

import math
from dataclasses import dataclass

import numpy as np

from ros_robot_studio.motion.schema import MotionSettings


@dataclass
class JointTrajectory:
    start: dict[str, float]
    end: dict[str, float]
    duration: float
    demo_limits: tuple[str, ...] = ()
    unchecked_accelerations: tuple[str, ...] = ()

    def sample(self, seconds: float):
        u = float(np.clip(seconds / self.duration, 0, 1)) if self.duration else 1.0
        blend = 3 * u * u - 2 * u * u * u
        return {
            name: value + (self.end[name] - value) * blend for name, value in self.start.items()
        }

    def peak_velocity(self, name):
        return (
            1.5 * abs(self.end[name] - self.start[name]) / self.duration if self.duration else 0.0
        )

    def peak_acceleration(self, name):
        return (
            6 * abs(self.end[name] - self.start[name]) / self.duration**2 if self.duration else 0.0
        )


class TrajectoryGenerator:
    def __init__(self, engine):
        self.engine, self.model = engine, engine.model

    def generate(self, start, end, settings: MotionSettings | None = None, minimum_duration=0):
        if not math.isfinite(minimum_duration) or minimum_duration < 0:
            raise ValueError("Minimum segment duration must be finite and nonnegative")
        settings = settings or MotionSettings()
        self.model.configuration(start)
        self.model.configuration(end)
        q0, q1 = self.engine.seed(start), self.engine.seed(end)
        bounds = self.engine.bounds()
        for name in q0:
            joint = self.engine.joints[name]
            if joint.kind == "continuous" and not np.isfinite(bounds[name]).any():
                delta = q1[name] - q0[name]
                q1[name] = q0[name] + math.atan2(math.sin(delta), math.cos(delta))
        full0, full1 = self.model.configuration(q0), self.model.configuration(q1)
        duration = max(settings.minimum_duration, minimum_duration)
        demo, unchecked = [], []
        for joint in self.model.joints:
            if not joint.movable:
                continue
            delta = abs(full1[joint.name] - full0[joint.name])
            if delta < 1e-12:
                continue
            velocity = joint.velocity
            if velocity is None:
                velocity = (
                    settings.demo_prismatic_velocity
                    if joint.kind == "prismatic"
                    else settings.demo_revolute_velocity
                )
                demo.append(joint.name)
            velocity *= settings.speed_scale
            if velocity <= 0:
                raise ValueError(f"{joint.name}: zero velocity limit prohibits this move")
            duration = max(duration, 1.5 * delta / velocity)
            acceleration = joint.acceleration
            override = settings.acceleration_limits.get(joint.name)
            if override is not None:
                acceleration = min(acceleration, override) if acceleration is not None else override
            if acceleration is not None:
                if acceleration <= 0:
                    raise ValueError(f"{joint.name}: zero acceleration limit prohibits this move")
                duration = max(duration, math.sqrt(6 * delta / acceleration))
            else:
                unchecked.append(joint.name)
        if not math.isfinite(duration) or duration > 3600:
            raise ValueError("Preview segment duration is invalid or exceeds one hour")
        result = JointTrajectory(q0, q1, duration, tuple(demo), tuple(unchecked))
        # Convex bounded interpolation plus explicit samples catches coupling/numerical errors.
        for t in np.linspace(0, duration, 33):
            self.model.configuration(result.sample(float(t)))
        # Analytic peaks include mimic derivatives; no physical limits are fabricated.
        for joint in self.model.joints:
            if not joint.movable:
                continue
            delta = abs(full1[joint.name] - full0[joint.name])
            velocity = (
                joint.velocity
                if joint.velocity is not None
                else settings.demo_prismatic_velocity
                if joint.kind == "prismatic"
                else settings.demo_revolute_velocity
            ) * settings.speed_scale
            if 1.5 * delta / duration > velocity + 1e-9:
                raise ValueError("Velocity validation failed")
            acceleration = joint.acceleration
            override = settings.acceleration_limits.get(joint.name)
            if override is not None:
                acceleration = min(acceleration, override) if acceleration is not None else override
            if acceleration is not None and 6 * delta / duration**2 > acceleration + 1e-9:
                raise ValueError("Acceleration validation failed")
        return result
