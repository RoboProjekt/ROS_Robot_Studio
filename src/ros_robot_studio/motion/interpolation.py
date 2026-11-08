"""Shared cubic interpolation primitives used by desktop preview and ROS controller."""


def cubic(u):
    return 3 * u * u - 2 * u * u * u


def cubic_derivatives(u, duration):
    return (
        ((6 * u - 6 * u * u) / duration, (6 - 12 * u) / duration**2) if duration > 0 else (0.0, 0.0)
    )


def hermite(p0, p1, v0, v1, u, duration):
    if duration <= 0:
        return p1, 0.0, 0.0
    a = 2 * p0 - 2 * p1 + duration * (v0 + v1)
    b = -3 * p0 + 3 * p1 - duration * (2 * v0 + v1)
    c = duration * v0
    return (
        ((a * u + b) * u + c) * u + p0,
        (3 * a * u * u + 2 * b * u + c) / duration,
        (6 * a * u + 2 * b) / duration**2,
    )
