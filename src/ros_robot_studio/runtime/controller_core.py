"""Pure ROS simulation trajectory validation/sampling, also exercised by desktop tests.

This module and interpolation.py are copied into the image. No ROS/Qt dependencies.
"""

import math

from ros_robot_studio.motion.interpolation import hermite


def stamp_seconds(stamp):
    return stamp["sec"] + stamp["nanosec"] * 1e-9


class SimulatedTrajectory:
    def __init__(self, message, model, current, start_tolerance=0.01):
        names = message["joint_names"]
        points = message["points"]
        expected = set(model.default_configuration())
        if len(names) != len(set(names)) or set(names) != expected:
            raise ValueError("Trajectory joint names do not match this robot")
        if not 2 <= len(points) <= 20001:
            raise ValueError("Trajectory requires 2..20001 points")
        previous = -1.0
        self.times = []
        self.points = points
        self.names = names
        self.model = model
        for point in points:
            stamp = point["time_from_start"]
            if (
                not isinstance(stamp.get("sec"), int)
                or not isinstance(stamp.get("nanosec"), int)
                or stamp["sec"] < 0
                or not 0 <= stamp["nanosec"] < 1_000_000_000
            ):
                raise ValueError("Invalid duration")
            t = stamp_seconds(stamp)
            if t <= previous or not math.isfinite(t) or t > 3600:
                raise ValueError("Trajectory times must increase")
            previous = t
            self.times.append(t)
            for field in ("positions", "velocities", "accelerations"):
                values = point.get(field, [])
                if len(values) != len(names) or not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in values
                ):
                    raise ValueError("Invalid " + field)
            model.configuration(dict(zip(names, point["positions"])))
        if self.times[0] != 0:
            raise ValueError("Trajectory must start at time zero")
        if any(
            abs(current[n] - v) > start_tolerance for n, v in zip(names, points[0]["positions"])
        ):
            raise ValueError("Start position mismatch; replan")
        self.duration = self.times[-1]
        # Validate every Hermite interval, including exact interior velocity/position extrema.
        for i in range(len(points) - 1):
            span = self.times[i + 1] - self.times[i]
            for k, name in enumerate(names):
                p0, p1 = points[i]["positions"][k], points[i + 1]["positions"][k]
                v0, v1 = points[i]["velocities"][k], points[i + 1]["velocities"][k]
                a = 2 * p0 - 2 * p1 + span * (v0 + v1)
                b = -3 * p0 + 3 * p1 - span * (2 * v0 + v1)
                c = span * v0
                us = {0.0, 1.0, 0.5}
                if abs(a) > 1e-14:
                    us.add(-b / (3 * a))
                    discriminant = 4 * b * b - 12 * a * c
                    if discriminant >= 0:
                        root = math.sqrt(discriminant)
                        us.update([(-2 * b + root) / (6 * a), (-2 * b - root) / (6 * a)])
                elif abs(b) > 1e-14:
                    us.add(-c / (2 * b))
                joint = model.joint_by_name[name]
                for u in us:
                    if not 0 <= u <= 1:
                        continue
                    p, v, acc = hermite(p0, p1, v0, v1, u, span)
                    q = dict(current, **{name: p})
                    model.configuration(q)
                    if joint.velocity is not None and abs(v) > joint.velocity + 1e-6:
                        raise ValueError("Velocity limit exceeded")
                    if joint.acceleration is not None and abs(acc) > joint.acceleration + 1e-5:
                        raise ValueError("Acceleration limit exceeded")
                    for coupled in model.joints:
                        if (
                            coupled.movable
                            and coupled.mimic
                            and model.driver(coupled.name)[0] == name
                        ):
                            factor = model.driver(coupled.name)[1]
                            if (
                                coupled.velocity is not None
                                and abs(factor * v) > coupled.velocity + 1e-6
                            ):
                                raise ValueError("Mimic velocity exceeded")
                            if (
                                coupled.acceleration is not None
                                and abs(factor * acc) > coupled.acceleration + 1e-5
                            ):
                                raise ValueError("Mimic acceleration exceeded")

    def sample(self, seconds):
        index = next(
            (i for i in range(len(self.times) - 1) if seconds < self.times[i + 1]),
            len(self.times) - 2,
        )
        span = self.times[index + 1] - self.times[index]
        u = max(0.0, min(1.0, (seconds - self.times[index]) / span))
        start, end = self.points[index], self.points[index + 1]
        q = {}
        v = {}
        a = {}
        for k, name in enumerate(self.names):
            q[name], v[name], a[name] = hermite(
                start["positions"][k],
                end["positions"][k],
                start["velocities"][k],
                end["velocities"][k],
                u,
                span,
            )
        return q, v, a
