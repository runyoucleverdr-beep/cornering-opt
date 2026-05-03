from __future__ import annotations

import numpy as np
from scipy.optimize import Bounds, NonlinearConstraint

from cornering.models.config import CornerModelConfig
from cornering.geometry.path_builder import PathBuilder
from cornering.optimization.objective import unpack_decision_vector


def build_variable_bounds(cfg: CornerModelConfig) -> Bounds:
    """
    Bounds for:
        s_turn, s_apex, s_exit, q_1, ..., q_N
    """
    lower = [
        cfg.s_turn_bounds[0],
        cfg.s_apex_bounds[0],
        cfg.s_exit_bounds[0],
    ] + [cfg.q_min] * cfg.num_path_points

    upper = [
        cfg.s_turn_bounds[1],
        cfg.s_apex_bounds[1],
        cfg.s_exit_bounds[1],
    ] + [np.inf] * cfg.num_path_points

    return Bounds(
    np.array(lower, dtype=float),
    np.array(upper, dtype=float),
)


def ordering_constraint_fun(x: np.ndarray, cfg: CornerModelConfig) -> np.ndarray:
    """
    Enforce:
        s_turn + eps <= s_apex
        s_apex + eps <= s_exit

    trust-constr form:
        fun(x) >= 0
    """
    s_turn, s_apex, s_exit, _ = unpack_decision_vector(x, cfg)
    eps = cfg.eps_order

    return np.array(
        [
            s_apex - s_turn - eps,
            s_exit - s_apex - eps,
        ],
        dtype=float,
    )


def track_width_constraint_fun(
    x: np.ndarray, cfg: CornerModelConfig, builder: PathBuilder
) -> np.ndarray:
    """
    Return lateral offset n_i(z).
    Constraint is enforced as:
        -limit <= n_i <= limit
    where
        limit = w + W_car / 2
    """
    s_turn, s_apex, s_exit, _ = unpack_decision_vector(x, cfg)
    path = builder.build_path(s_turn, s_apex, s_exit)
    return np.asarray(path.lateral_offset, dtype=float)


def friction_constraint_fun(
    x: np.ndarray, cfg: CornerModelConfig, builder: PathBuilder
) -> np.ndarray:
    """
    Use squared friction form:
        q_i^2 * kappa_i(z)^2 <= (mu g)^2
    """
    s_turn, s_apex, s_exit, q = unpack_decision_vector(x, cfg)
    path = builder.build_path(s_turn, s_apex, s_exit)

    kappa_sq = np.square(path.curvature)
    return np.square(q) * kappa_sq


def acceleration_constraint_fun(
    x: np.ndarray, cfg: CornerModelConfig
) -> np.ndarray:
    """
    Acceleration constraint:
        q_{i+1} - q_i <= 2 a_acc_max ds

    Since ds depends on the path, we handle that in a wrapper constraint below.
    This function only returns q_{i+1} - q_i.
    """
    _, _, _, q = unpack_decision_vector(x, cfg)
    return q[1:] - q[:-1]


def braking_constraint_fun(
    x: np.ndarray, cfg: CornerModelConfig
) -> np.ndarray:
    """
    Braking constraint:
        q_i - q_{i+1} <= 2 a_brake_max ds
    """
    _, _, _, q = unpack_decision_vector(x, cfg)
    return q[:-1] - q[1:]


def entry_speed_constraint_fun(x: np.ndarray, cfg: CornerModelConfig) -> np.ndarray:
    """
    Enforce:
        q_1 = v_in^2
    """
    _, _, _, q = unpack_decision_vector(x, cfg)
    return np.array([q[0]], dtype=float)


def build_constraints(cfg: CornerModelConfig, builder: PathBuilder):
    constraints = []

    # 1) ordering: >= 0
    constraints.append(
        NonlinearConstraint(
            fun=lambda x: ordering_constraint_fun(x, cfg),
            lb=0.0,
            ub=np.inf,
        )
    )

    # 2) track width: -limit <= n_i <= limit
    track_limit = cfg.track_half_width + 0.5 * cfg.car_width
    constraints.append(
        NonlinearConstraint(
            fun=lambda x: track_width_constraint_fun(x, cfg, builder),
            lb=-track_limit,
            ub=track_limit,
        )
    )

    # 3) friction: q_i^2 * kappa_i^2 <= (mu g)^2
    friction_upper = np.full(cfg.num_path_points, (cfg.mu * cfg.g) ** 2, dtype=float)
    constraints.append(
        NonlinearConstraint(
            fun=lambda x: friction_constraint_fun(x, cfg, builder),
            lb=-np.inf,
            ub=friction_upper,
        )
    )

    # 4) acceleration: q_{i+1} - q_i - 2 a_acc_max ds <= 0
    def accel_residual(x: np.ndarray) -> np.ndarray:
        s_turn, s_apex, s_exit, _ = unpack_decision_vector(x, cfg)
        path = builder.build_path(s_turn, s_apex, s_exit)
        dq = acceleration_constraint_fun(x, cfg)
        return dq - 2.0 * cfg.a_acc_max * path.ds_mean

    constraints.append(
        NonlinearConstraint(
            fun=accel_residual,
            lb=-np.inf,
            ub=0.0,
        )
    )

    # 5) braking: q_i - q_{i+1} - 2 a_brake_max ds <= 0
    def brake_residual(x: np.ndarray) -> np.ndarray:
        s_turn, s_apex, s_exit, _ = unpack_decision_vector(x, cfg)
        path = builder.build_path(s_turn, s_apex, s_exit)
        dq = braking_constraint_fun(x, cfg)
        return dq - 2.0 * cfg.a_brake_max * path.ds_mean

    constraints.append(
        NonlinearConstraint(
            fun=brake_residual,
            lb=-np.inf,
            ub=0.0,
        )
    )

    # 6) entry speed: q_1 = v_in^2
    constraints.append(
        NonlinearConstraint(
            fun=lambda x: entry_speed_constraint_fun(x, cfg),
            lb=cfg.v_in ** 2,
            ub=cfg.v_in ** 2,
        )
    )

    return constraints

def diagnose_constraints(x: np.ndarray, cfg: CornerModelConfig, builder: PathBuilder) -> dict:
    """
    Return a dictionary of per-constraint diagnostics.
    Each item contains:
      - raw values
      - max violation
    """
    s_turn, s_apex, s_exit, q = unpack_decision_vector(x, cfg)
    path = builder.build_path(s_turn, s_apex, s_exit)

    # 1) Ordering
    ordering_vals = ordering_constraint_fun(x, cfg)   # should be >= 0
    ordering_violation = np.maximum(0.0, -ordering_vals)

    # 2) Track width
    n = np.asarray(path.lateral_offset, dtype=float)
    track_limit = cfg.track_half_width + 0.5 * cfg.car_width
    track_violation = np.maximum(0.0, np.abs(n) - track_limit)

    # 3) Friction
    friction_vals = friction_constraint_fun(x, cfg, builder)
    friction_limit = (cfg.mu * cfg.g) ** 2
    friction_violation = np.maximum(0.0, friction_vals - friction_limit)

    # 4) Acceleration
    accel_vals = acceleration_constraint_fun(x, cfg) - 2.0 * cfg.a_acc_max * path.ds_mean
    accel_violation = np.maximum(0.0, accel_vals)

    # 5) Braking
    brake_vals = braking_constraint_fun(x, cfg) - 2.0 * cfg.a_brake_max * path.ds_mean
    brake_violation = np.maximum(0.0, brake_vals)

    # 6) Entry speed
    entry_val = q[0]
    entry_target = cfg.v_in ** 2
    entry_violation = np.array([abs(entry_val - entry_target)], dtype=float)

    # 7) Variable bounds
    q_lower_violation = np.maximum(0.0, cfg.q_min - q)

    s_turn_lower = max(0.0, cfg.s_turn_bounds[0] - s_turn)
    s_turn_upper = max(0.0, s_turn - cfg.s_turn_bounds[1])

    s_apex_lower = max(0.0, cfg.s_apex_bounds[0] - s_apex)
    s_apex_upper = max(0.0, s_apex - cfg.s_apex_bounds[1])

    s_exit_lower = max(0.0, cfg.s_exit_bounds[0] - s_exit)
    s_exit_upper = max(0.0, s_exit - cfg.s_exit_bounds[1])

    bounds_violation = np.array(
        [
            s_turn_lower, s_turn_upper,
            s_apex_lower, s_apex_upper,
            s_exit_lower, s_exit_upper,
            float(np.max(q_lower_violation)),
        ],
        dtype=float,
    )

    return {
        "ordering": {
            "values": ordering_vals,
            "max_violation": float(np.max(ordering_violation)),
        },
        "track_width": {
            "values": n,
            "max_violation": float(np.max(track_violation)),
            "limit": float(track_limit),
        },
        "friction": {
            "values": friction_vals,
            "max_violation": float(np.max(friction_violation)),
            "limit": float(friction_limit),
        },
        "acceleration": {
            "values": accel_vals,
            "max_violation": float(np.max(accel_violation)),
        },
        "braking": {
            "values": brake_vals,
            "max_violation": float(np.max(brake_violation)),
        },
        "entry_speed": {
            "value": float(entry_val),
            "target": float(entry_target),
            "max_violation": float(np.max(entry_violation)),
        },
        "bounds": {
            "values": bounds_violation,
            "max_violation": float(np.max(bounds_violation)),
            "details": {
                "s_turn_lower": float(s_turn_lower),
                "s_turn_upper": float(s_turn_upper),
                "s_apex_lower": float(s_apex_lower),
                "s_apex_upper": float(s_apex_upper),
                "s_exit_lower": float(s_exit_lower),
                "s_exit_upper": float(s_exit_upper),
                "q_lower": float(np.max(q_lower_violation)),
            },
        },
    }