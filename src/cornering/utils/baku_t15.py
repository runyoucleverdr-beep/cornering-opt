from __future__ import annotations

import numpy as np
from cornering.models.config import CornerModelConfig


def _integrate_centerline_from_curvature(
    total_length: float = 72.0,
    ds: float = 0.25,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    s = np.arange(0.0, total_length + ds, ds)
    kappa = np.zeros_like(s)

    # Segment A: approach straight
    # 0 - 24 m

    # Segment B: short rightward kink under braking
    mask_b = (s >= 24.0) & (s < 30.0)
    kappa[mask_b] = -0.014

    # Segment C: main left arc
    mask_c = (s >= 30.0) & (s < 40.0)
    kappa[mask_c] = 0.115

    # Segment D: short unwind
    mask_d = (s >= 40.0) & (s < 48.0)
    kappa[mask_d] = 0.040

    x = np.zeros_like(s)
    y = np.zeros_like(s)
    heading = np.zeros_like(s)

    for i in range(1, len(s)):
        heading[i] = heading[i - 1] + kappa[i - 1] * ds
        x[i] = x[i - 1] + np.cos(heading[i - 1]) * ds
        y[i] = y[i - 1] + np.sin(heading[i - 1]) * ds

    centerline_xy = np.column_stack([x, y])
    return s, centerline_xy, heading


def build_baku_t15_config() -> CornerModelConfig:
    centerline_s, centerline_xy, centerline_heading = _integrate_centerline_from_curvature(
        total_length=72.0,
        ds=0.25,
    )

    track_half_width = 6.0
    corner_direction = "left"

    entry_heading = centerline_heading[0]
    entry_normal = np.array(
        [-np.sin(entry_heading), np.cos(entry_heading)],
        dtype=float,
    )
    entry_center = centerline_xy[0]
    entry_point = entry_center - track_half_width * entry_normal

    exit_heading = centerline_heading[-1]
    exit_normal = np.array(
        [-np.sin(exit_heading), np.cos(exit_heading)],
        dtype=float,
    )
    exit_center = centerline_xy[-1]
    exit_point = exit_center - track_half_width * exit_normal

    cfg = CornerModelConfig(
        track_half_width=track_half_width,
        car_width=2.0,
        entry_point=(float(entry_point[0]), float(entry_point[1])),
        exit_point=(float(exit_point[0]), float(exit_point[1])),
        centerline_s=centerline_s,
        centerline_xy=centerline_xy,
        centerline_heading=centerline_heading,
        num_path_points=20,
        eps_order=1.0,
        q_min=1e-2,
        mu=1.4,
        g=9.81,
        a_acc_max=5.0,
        a_brake_max=9.0,
        v_in=22.0,
        s_turn_bounds=(18.0, 28.0),
        s_apex_bounds=(30.0, 40.0),
        s_exit_bounds=(40.0, 58.0),
        boundary_margin=0.4,
        corner_direction=corner_direction,
    )
    cfg.validate()
    return cfg