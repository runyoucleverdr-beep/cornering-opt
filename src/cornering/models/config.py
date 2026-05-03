from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class CornerModelConfig:
    # Track geometry
    track_half_width: float
    car_width: float
    entry_point: Tuple[float, float]
    exit_point: Tuple[float, float]
    centerline_s: np.ndarray
    centerline_xy: np.ndarray          # shape (M, 2)
    centerline_heading: np.ndarray     # shape (M,)

    # Discretization
    num_path_points: int = 30
    eps_order: float = 1.0
    q_min: float = 1e-2

    # Physics
    mu: float = 1.2
    g: float = 9.81
    a_acc_max: float = 6.0
    a_brake_max: float = 10.0
    v_in: float = 20.0

    # Bounds for geometric variables
    s_turn_bounds: Tuple[float, float] = (5.0, 20.0)
    s_apex_bounds: Tuple[float, float] = (20.0, 40.0)
    s_exit_bounds: Tuple[float, float] = (40.0, 60.0)
    boundary_margin: float = 0.4
    
    # Corner direction: "right" or "left"
    corner_direction: str = "right"

    def validate(self) -> None:
        if self.centerline_xy.ndim != 2 or self.centerline_xy.shape[1] != 2:
            raise ValueError("centerline_xy must have shape (M, 2).")

        if self.centerline_s.ndim != 1:
            raise ValueError("centerline_s must be a 1D array.")

        if self.centerline_heading.ndim != 1:
            raise ValueError("centerline_heading must be a 1D array.")

        if not (
            len(self.centerline_s)
            == len(self.centerline_xy)
            == len(self.centerline_heading)
        ):
            raise ValueError("centerline_s, centerline_xy, and centerline_heading must have the same length.")

        if self.corner_direction not in {"right", "left"}:
            raise ValueError("corner_direction must be 'right' or 'left'.")