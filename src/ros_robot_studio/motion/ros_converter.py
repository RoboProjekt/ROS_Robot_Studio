"""Convert already planned cubic segments to ROS messages; never plan a second path."""

import math


def duration(seconds):
    nanoseconds = round(seconds * 1e9)
    return {"sec": nanoseconds // 1_000_000_000, "nanosec": nanoseconds % 1_000_000_000}


class TrajectoryToROSConverter:
    def convert(self, trajectory, model, mapping=None, period=0.05):
        if not math.isfinite(trajectory.duration) or trajectory.duration <= 0:
            raise ValueError("Trajectory duration must be positive")
        mapping = mapping or {n: n for n in trajectory.start}
        names = list(trajectory.start)
        if set(mapping) != set(names) or len(set(mapping.values())) != len(mapping):
            raise ValueError("Complete one-to-one joint mapping required")
        if set(names) != set(model.default_configuration()):
            raise ValueError("Trajectory does not cover the independent robot joints")
        points = []
        count = max(1, math.ceil(trajectory.duration / period))
        if count > 20000:
            raise ValueError("Trajectory exceeds bounded point count")
        for i in range(count + 1):
            seconds = trajectory.duration * i / count
            q = trajectory.sample(seconds)
            model.configuration(q)
            velocities = trajectory.velocity(seconds)
            accelerations = trajectory.acceleration(seconds)
            if not all(
                math.isfinite(v)
                for values in (q, velocities, accelerations)
                for v in values.values()
            ):
                raise ValueError("Nonfinite trajectory")
            points.append(
                dict(
                    positions=[q[n] for n in names],
                    velocities=[velocities[n] for n in names],
                    accelerations=[accelerations[n] for n in names],
                    time_from_start=duration(seconds),
                )
            )
        return dict(
            header={"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            joint_names=[mapping[n] for n in names],
            points=points,
        )
