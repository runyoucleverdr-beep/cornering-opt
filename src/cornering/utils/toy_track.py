from __future__ import annotations

import numpy as np
from cornering.models.config import CornerModelConfig


def build_toy_corner_config() -> CornerModelConfig:
    """
    Build a toy left-hand corner:
    - short entry straight
    - quarter-circle left turn
    - short exit straight
    """
    s = np.linspace(0.0, 60.0, 240)
    x = np.zeros_like(s)
    y = np.zeros_like(s)
    heading = np.zeros_like(s)

    R = 20.0

    for i, si in enumerate(s):
        if si < 15.0:
            # Entry straight
            x[i] = si
            y[i] = 0.0
            heading[i] = 0.0

        elif si < 45.0:
            # Quarter-circle left turn
            theta = (si - 15.0) / 30.0 * (np.pi / 2.0)
            x[i] = 15.0 + R * np.sin(theta)
            y[i] = R * (1.0 - np.cos(theta))
            heading[i] = theta

        else:
            # Exit straight
            x[i] = 15.0 + R
            y[i] = R + (si - 45.0)
            heading[i] = np.pi / 2.0

    centerline_xy = np.column_stack([x, y])

    # For a LEFT corner:
    # outside = right side of the car = negative normal offset
    track_half_width = 6.0

    entry_heading = heading[0]
    entry_normal = np.array([-np.sin(entry_heading), np.cos(entry_heading)], dtype=float)
    entry_point = np.array([x[0], y[0]], dtype=float) - track_half_width * entry_normal

    exit_heading = heading[-1]
    exit_normal = np.array([-np.sin(exit_heading), np.cos(exit_heading)], dtype=float)
    exit_point = np.array([x[-1], y[-1]], dtype=float) - track_half_width * exit_normal

    cfg = CornerModelConfig(
        track_half_width=track_half_width,
        car_width=2.0,
        entry_point=(float(entry_point[0]), float(entry_point[1])),
        exit_point=(float(exit_point[0]), float(exit_point[1])),
        centerline_s=s,
        centerline_xy=centerline_xy,
        centerline_heading=heading,
        num_path_points=20,
        mu=1.4,
        g=9.81,
        a_acc_max=5.0,
        a_brake_max=9.0,
        v_in=18.0,
        s_turn_bounds=(8.0, 20.0),
        s_apex_bounds=(20.0, 38.0),
        s_exit_bounds=(38.0, 55.0),
        corner_direction="left",
    )
    cfg.validate()
    return cfg