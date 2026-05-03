from __future__ import annotations

import numpy as np
from scipy.optimize import minimize

from cornering.models.config import CornerModelConfig
from cornering.geometry.path_builder import PathBuilder
from cornering.optimization.objective import objective_time
from cornering.optimization.constraints import (
    build_variable_bounds,
    build_constraints,
)


from cornering.geometry.path_builder import PathBuilder


def build_initial_guess(cfg: CornerModelConfig) -> np.ndarray:
    """
    Initial guess:
    - geometric vars chosen manually but clipped strictly inside bounds
    - q profile initialized from curvature-based safe speeds
    """
    # desired manual guesses
    s_turn0_raw = 20.0
    s_apex0_raw = 35.0
    s_exit0_raw = 50.0

    # keep the initial point strictly inside bounds
    margin = 0.2

    s_turn0 = np.clip(
        s_turn0_raw,
        cfg.s_turn_bounds[0] + margin,
        cfg.s_turn_bounds[1] - margin,
    )
    s_apex0 = np.clip(
        s_apex0_raw,
        cfg.s_apex_bounds[0] + margin,
        cfg.s_apex_bounds[1] - margin,
    )
    s_exit0 = np.clip(
        s_exit0_raw,
        cfg.s_exit_bounds[0] + margin,
        cfg.s_exit_bounds[1] - margin,
    )

    builder = PathBuilder(cfg)
    path0 = builder.build_path(s_turn0, s_apex0, s_exit0)

    kappa_abs = np.maximum(np.abs(path0.curvature), 1e-6)
    q_cap = (cfg.mu * cfg.g) / kappa_abs

    q0 = 0.6 * q_cap
    q0 = np.minimum(q0, cfg.v_in ** 2)
    q0 = np.maximum(q0, cfg.q_min + 1e-6)

    # enforce entry-speed equality exactly
    q0[0] = cfg.v_in ** 2

    x0 = np.concatenate([[s_turn0, s_apex0, s_exit0], q0])
    return x0


def solve_single_level(cfg: CornerModelConfig, x0: np.ndarray | None = None):
    cfg.validate()
    builder = PathBuilder(cfg)

    if x0 is None:
        x0 = build_initial_guess(cfg)

    bounds = build_variable_bounds(cfg)
    
    print("\n=== Initial Guess Check ===")
    print("x0[:3] =", x0[:3])
    print("geom lower =", bounds.lb[:3])
    print("geom upper =", bounds.ub[:3])
    print("q min lower bound =", bounds.lb[3])
    print("q min actual =", np.min(x0[3:]))

    if np.any(x0 < bounds.lb) or np.any(x0 > bounds.ub):
        idx_low = np.where(x0 < bounds.lb)[0]
        idx_up = np.where(x0 > bounds.ub)[0]
        print("Below lower bound indices:", idx_low)
        print("Above upper bound indices:", idx_up)
        raise ValueError("Initial guess x0 is outside bounds before calling minimize.")
    constraints = build_constraints(cfg, builder)

    result = minimize(
        fun=lambda x: objective_time(x, cfg, builder),
        x0=x0,
        method="trust-constr",
        bounds=bounds,
        constraints=constraints,
        options={
            "verbose": 3,
            "maxiter": 300,
            "gtol": 1e-6,
            "xtol": 1e-8,
            "barrier_tol": 1e-8,
        },
    )
    return result

def project_to_basic_feasible_guess(x: np.ndarray, cfg: CornerModelConfig) -> np.ndarray:
    x_proj = x.copy()

    # Project geometric variables into bounds, but keep them slightly inside.
    margin = 0.2
    x_proj[0] = np.clip(x_proj[0], cfg.s_turn_bounds[0] + margin, cfg.s_turn_bounds[1] - margin)
    x_proj[1] = np.clip(x_proj[1], cfg.s_apex_bounds[0] + margin, cfg.s_apex_bounds[1] - margin)
    x_proj[2] = np.clip(x_proj[2], cfg.s_exit_bounds[0] + margin, cfg.s_exit_bounds[1] - margin)

    # Enforce entry-speed equality exactly.
    x_proj[3] = cfg.v_in ** 2

    # Enforce q lower bound.
    x_proj[3:] = np.maximum(x_proj[3:], cfg.q_min)

    return x_proj