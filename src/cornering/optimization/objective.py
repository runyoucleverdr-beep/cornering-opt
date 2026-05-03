from __future__ import annotations

import numpy as np

from cornering.models.config import CornerModelConfig
from cornering.geometry.path_builder import PathBuilder


def unpack_decision_vector(x: np.ndarray, cfg: CornerModelConfig):
    """
    x = [s_turn, s_apex, s_exit, q_1, ..., q_N]
    where q_i = v_i^2
    """
    if len(x) != 3 + cfg.num_path_points:
        raise ValueError(
            f"Decision vector length mismatch: got {len(x)}, expected {3 + cfg.num_path_points}."
        )

    s_turn = float(x[0])
    s_apex = float(x[1])
    s_exit = float(x[2])
    q = np.asarray(x[3:], dtype=float)
    return s_turn, s_apex, s_exit, q


def objective_time(x: np.ndarray, cfg: CornerModelConfig, builder: PathBuilder) -> float:
    """
    Minimum-time objective:
        T(x) ~= sum(ds / sqrt(q_i))
    """
    s_turn, s_apex, s_exit, q = unpack_decision_vector(x, cfg)

    path = builder.build_path(s_turn, s_apex, s_exit)

    q_safe = np.maximum(q, cfg.q_min)
    return float(np.sum(path.ds_mean / np.sqrt(q_safe)))