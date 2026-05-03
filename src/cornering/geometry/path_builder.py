from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from cornering.models.config import CornerModelConfig


@dataclass
class PathData:
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    curvature: np.ndarray
    s_path: np.ndarray
    ds_mean: float
    path_length: float
    lateral_offset: np.ndarray
    control_points: np.ndarray


class PathBuilder:
    def __init__(self, cfg: CornerModelConfig):
        self.cfg = cfg

        s_ref = cfg.centerline_s
        xy_ref = cfg.centerline_xy
        hdg_ref = cfg.centerline_heading

        self.cx = CubicSpline(s_ref, xy_ref[:, 0], bc_type="natural")
        self.cy = CubicSpline(s_ref, xy_ref[:, 1], bc_type="natural")
        self.ch = CubicSpline(s_ref, hdg_ref, bc_type="natural")

    def centerline_point_heading(self, s: float) -> Tuple[np.ndarray, float]:
        p = np.array([self.cx(s), self.cy(s)], dtype=float)
        heading = float(self.ch(s))
        return p, heading

    def offset_point(self, s: float, lateral_offset: float) -> np.ndarray:
        """
        Offset from centerline using the local normal.
        Positive offset means left of the centerline tangent.
        """
        p, heading = self.centerline_point_heading(s)
        normal = np.array([-np.sin(heading), np.cos(heading)], dtype=float)
        return p + lateral_offset * normal

    def _outside_inside_offsets(self) -> tuple[float, float]:
        effective_w = max(self.cfg.track_half_width - self.cfg.boundary_margin, 0.1)

        if self.cfg.corner_direction == "right":
            outside = +effective_w
            inside = -effective_w
        else:
            outside = -effective_w
            inside = +effective_w

        return outside, inside

    def build_control_points(
        self, s_turn: float, s_apex: float, s_exit: float
    ) -> np.ndarray:
        outside, inside = self._outside_inside_offsets()

        p0 = np.array(self.cfg.entry_point, dtype=float)
        p1 = self.offset_point(s_turn, outside)
        p2 = self.offset_point(s_apex, inside)
        p3 = self.offset_point(s_exit, outside)
        p4 = np.array(self.cfg.exit_point, dtype=float)

        cps = np.vstack([p0, p1, p2, p3, p4])
        return cps

    def build_path(
        self, s_turn: float, s_apex: float, s_exit: float
    ) -> PathData:
        cps = self.build_control_points(s_turn, s_apex, s_exit)

        # Parameterize control points by cumulative chord length
        seg = np.linalg.norm(np.diff(cps, axis=0), axis=1)
        u = np.concatenate([[0.0], np.cumsum(seg)])

        if np.any(np.diff(u) <= 1e-10):
            raise ValueError("Degenerate control points: repeated spline knot values.")

        sx = CubicSpline(u, cps[:, 0], bc_type="natural")
        sy = CubicSpline(u, cps[:, 1], bc_type="natural")

        u_eval = np.linspace(u[0], u[-1], self.cfg.num_path_points)

        x = sx(u_eval)
        y = sy(u_eval)

        dx = sx(u_eval, 1)
        dy = sy(u_eval, 1)
        ddx = sx(u_eval, 2)
        ddy = sy(u_eval, 2)

        heading = np.arctan2(dy, dx)

        denom = np.power(dx * dx + dy * dy, 1.5)
        denom = np.maximum(denom, 1e-8)
        curvature = (dx * ddy - dy * ddx) / denom

        ds_seg = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
        s_path = np.concatenate([[0.0], np.cumsum(ds_seg)])
        path_length = float(s_path[-1])
        ds_mean = path_length / max(len(x) - 1, 1)

        lateral_offset = self.compute_lateral_offset(x, y)

        return PathData(
            x=x,
            y=y,
            heading=heading,
            curvature=curvature,
            s_path=s_path,
            ds_mean=ds_mean,
            path_length=path_length,
            lateral_offset=lateral_offset,
            control_points=cps,
        )

    def compute_lateral_offset(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Approximate lateral offset by projection to the nearest sampled centerline point.
        This is a simple first version and is good enough for the toy case.
        """
        ref_xy = self.cfg.centerline_xy
        ref_heading = self.cfg.centerline_heading

        n = np.zeros_like(x, dtype=float)

        for i in range(len(x)):
            p = np.array([x[i], y[i]], dtype=float)
            j = np.argmin(np.sum((ref_xy - p) ** 2, axis=1))

            delta = p - ref_xy[j]
            normal = np.array(
                [-np.sin(ref_heading[j]), np.cos(ref_heading[j])],
                dtype=float,
            )
            n[i] = float(delta @ normal)

        return n

    def check_ordering(self, s_turn: float, s_apex: float, s_exit: float) -> bool:
        eps = self.cfg.eps_order
        return (s_turn + eps <= s_apex) and (s_apex + eps <= s_exit)

    def check_track_width(self, path: PathData) -> bool:
        limit = self.cfg.track_half_width + 0.5 * self.cfg.car_width
        return bool(np.all(np.abs(path.lateral_offset) <= limit + 1e-8))