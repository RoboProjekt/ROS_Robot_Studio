"""Optional SciPy trust-region IK, independently checked through the shared FK."""

from __future__ import annotations

import importlib.util
import math
import time
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from ros_robot_studio.motion.schema import CartesianPose, IKOptions


@dataclass(frozen=True)
class SolverDescriptor:
    id: str
    name: str
    installed: bool
    implemented: bool
    modes: tuple[str, ...] = ()


@dataclass
class IKCandidate:
    joints: dict[str, float]
    position_error: float
    orientation_error: float
    distance: float
    rank: int
    condition: float
    score: float
    converged: bool
    valid: bool
    evaluations: int


@dataclass
class IKResult:
    status: str
    message: str
    best: IKCandidate | None = None
    candidates: list[IKCandidate] = field(default_factory=list)
    solver: str = "scipy_trf"

    @property
    def success(self):
        return self.status == "exact" and self.best is not None and self.best.valid

    @property
    def joint_positions(self):
        return dict(self.best.joints) if self.success else None


class IKSolver(Protocol):
    def solve(
        self, chain, target_pose, initial_guess=None, mode="position_only", **kwargs
    ) -> IKResult: ...


class KinematicsSolverManager:
    def __init__(self):
        self.backends: dict[str, IKSolver] = {"scipy_trf": ScipyLeastSquaresSolver()}

    def catalog(self):
        return [
            SolverDescriptor(
                "scipy_trf",
                "SciPy 1.15.3 · bounded least squares",
                importlib.util.find_spec("scipy") is not None,
                True,
                ("position_only", "axis", "full_pose"),
            )
        ] + [
            SolverDescriptor(name, title, False, False)
            for name, title in [
                ("moveit2", "MoveIt 2 · planned"),
                ("kdl", "KDL · planned"),
                ("trac_ik", "TRAC-IK · planned"),
                ("pinocchio", "Pinocchio · planned"),
                ("ikpy", "IKPy · evaluated, not integrated"),
            ]
        ]

    def register(self, identifier: str, solver: IKSolver):
        self.backends[identifier] = solver

    def solve(self, chain, target_pose, initial_guess=None, solver="auto", **kwargs):
        identifier = "scipy_trf" if solver == "auto" else solver
        if identifier not in self.backends:
            return IKResult("unsupported", f"Solver {identifier} is not implemented")
        if identifier == "scipy_trf" and importlib.util.find_spec("scipy") is None:
            return IKResult(
                "unavailable", "Install Universal Kinematics Support in the setup assistant"
            )
        return self.backends[identifier].solve(chain, target_pose, initial_guess, **kwargs)


class ScipyLeastSquaresSolver:
    def solve(
        self,
        chain,
        target_pose,
        initial_guess=None,
        mode="position_only",
        options=None,
        axis="Z",
        cancel=None,
    ) -> IKResult:
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        options = options or IKOptions()
        if mode not in ("position_only", "axis", "full_pose") or axis not in ("X", "Y", "Z"):
            return IKResult("unsupported", "Unknown IK mode or orientation axis")
        if mode == "full_pose" and not chain.analyze().full_pose_available:
            return IKResult(
                "unsupported",
                "This chain does not provide six independently controllable Cartesian dimensions",
            )
        try:
            original = chain.engine.seed(initial_guess)
            if isinstance(target_pose, CartesianPose):
                target = chain.target_matrix(target_pose, original)
                axis = target_pose.axis
            else:
                target = np.asarray(target_pose, dtype=float)
            if target.shape != (4, 4) or not np.isfinite(target).all():
                raise ValueError("Target must be a finite homogeneous 4×4 pose")
            if (
                not np.allclose(target[3], [0, 0, 0, 1], atol=1e-9)
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(target[:3, :3]), 1, atol=1e-7)
            ):
                raise ValueError("Target rotation is not a rigid, orthonormal transform")
        except (ValueError, TypeError) as exc:
            return IKResult("failed", str(exc))
        names = chain.active_joints
        axis_index = "XYZ".index(axis)
        deadline = time.monotonic() + options.time_limit
        candidates = []

        def configuration(x):
            return dict(original, **dict(zip(names, map(float, x))))

        def residual(x):
            if cancel is not None and cancel.is_set():
                raise InterruptedError("IK cancelled")
            if time.monotonic() > deadline:
                raise TimeoutError("IK search time limit reached")
            actual = chain.forward_kinematics(configuration(x))
            position = (actual[:3, 3] - target[:3, 3]) * options.position_weight
            if mode == "position_only":
                return position
            orientation = (
                (actual[:3, axis_index] - target[:3, axis_index])
                if mode == "axis"
                else Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            )
            return np.r_[position, orientation * options.orientation_weight]

        def assess(x, converged, evaluations):
            q = configuration(x)
            chain.model.configuration(q)
            actual = chain.forward_kinematics(q)
            pe = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
            oe = (
                float(
                    np.arccos(
                        np.clip(np.dot(actual[:3, axis_index], target[:3, axis_index]), -1, 1)
                    )
                )
                if mode == "axis"
                else float(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude())
                if mode == "full_pose"
                else 0.0
            )
            distances = []
            for i, name in enumerate(names):
                delta = q[name] - original[name]
                if (
                    chain.engine.joints[name].kind == "continuous"
                    and not np.isfinite(chain.lower[i])
                    and not np.isfinite(chain.upper[i])
                ):
                    delta = math.atan2(math.sin(delta), math.cos(delta))
                scale = chain.upper[i] - chain.lower[i]
                distances.append(
                    (delta / (scale if np.isfinite(scale) and scale > 0 else 2 * math.pi)) ** 2
                )
            distance = math.sqrt(sum(distances))
            rank, condition, _ = chain.conditioning(q, mode, axis)
            penalty = 0.005 * math.log10(max(condition, 1)) if math.isfinite(condition) else 0.1
            valid = (
                converged
                and pe <= options.position_tolerance
                and oe <= options.orientation_tolerance
            )
            score = (
                distance
                + penalty
                + 0.0001 * (pe / options.position_tolerance + oe / options.orientation_tolerance)
            )
            return IKCandidate(
                q, pe, oe, distance, rank, condition, score, converged, valid, evaluations
            )

        if not names:
            candidate = assess([], True, 0)
            return IKResult(
                "exact" if candidate.valid else "not_reached",
                "Chain has no active axes",
                candidate,
                [candidate],
            )
        lo, hi = chain.lower, chain.upper
        first = np.array([original[n] for n in names])
        rng = np.random.default_rng(options.seed)
        seeds = [first]
        for _ in range(options.starts - 1):
            lower = np.where(np.isfinite(lo), lo, first - math.pi)
            upper = np.where(np.isfinite(hi), hi, first + math.pi)
            seeds.append(rng.uniform(lower, upper))
        stopped = ""
        for seed in seeds:
            try:
                result = least_squares(
                    residual,
                    np.clip(seed, lo, hi),
                    bounds=(lo, hi),
                    method="trf",
                    jac="2-point",
                    x_scale="jac",
                    max_nfev=options.max_evaluations,
                    ftol=1e-10,
                    xtol=1e-10,
                    gtol=1e-10,
                )
                candidate = assess(result.x, bool(result.success), int(result.nfev))
                if not any(
                    np.allclose(list(c.joints.values()), list(candidate.joints.values()), atol=1e-6)
                    for c in candidates
                ):
                    candidates.append(candidate)
                if candidate.valid and candidate.distance < 1e-8:
                    break
            except InterruptedError:
                return IKResult("cancelled", "IK cancelled; no pose applied", candidates=candidates)
            except TimeoutError as exc:
                stopped = str(exc)
                break
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                stopped = str(exc)
        valid = sorted((c for c in candidates if c.valid), key=lambda c: c.score)
        if valid:
            best = valid[0]
            message = f"Exact within tolerances · {best.position_error * 1000:.5f} mm · {math.degrees(best.orientation_error):.5f}° · {len(valid)} valid candidate(s)"
            if not math.isfinite(best.condition) or best.condition > 1e5:
                message += " · singular / poorly conditioned at this pose"
            return IKResult(
                "exact", message, best, sorted(candidates, key=lambda c: (not c.valid, c.score))
            )
        best = (
            min(
                candidates,
                key=lambda c: c.position_error + options.orientation_weight * c.orientation_error,
            )
            if candidates
            else None
        )
        message = "No converged solution within the requested tolerances; no joint values applied"
        if best:
            message += f". Best residual: {best.position_error * 1000:.3f} mm, {math.degrees(best.orientation_error):.3f}°"
        if stopped:
            message += ". " + stopped
        return IKResult("not_reached", message, best, candidates)
