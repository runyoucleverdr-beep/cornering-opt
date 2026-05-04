from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from cornering.utils.baku_t15 import build_baku_t15_config
from cornering.geometry.path_builder import PathBuilder
from cornering.optimization.solver import (
    solve_single_level,
    project_to_basic_feasible_guess,
)
from cornering.optimization.constraints import diagnose_constraints


RESULTS_DIR = Path("results")
RUNS_DIR = RESULTS_DIR / "runs"
FIG_DIR = RESULTS_DIR / "figures" / "baku_t15_speed_sweep"


def ensure_dirs() -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def build_cfg_for_speed(v_in: float):
    cfg = build_baku_t15_config()
    cfg.v_in = float(v_in)
    cfg.validate()
    return cfg


def warm_start_from_previous(prev_x: np.ndarray, cfg):
    """
    Reuse previous solution as the next initial guess,
    but enforce the new entry-speed equality and basic feasibility projection.
    """
    x0 = prev_x.copy()
    x0[3] = cfg.v_in ** 2
    x0 = project_to_basic_feasible_guess(x0, cfg)
    return x0


def plot_track_and_path(cfg, path, v_in: float, save_path: Path) -> None:
    centerline = cfg.centerline_xy
    heading = cfg.centerline_heading
    w = cfg.track_half_width

    normals = np.column_stack([-np.sin(heading), np.cos(heading)])
    left_boundary = centerline + w * normals
    right_boundary = centerline - w * normals

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(centerline[:, 0], centerline[:, 1], "--", label="Centerline")
    ax.plot(left_boundary[:, 0], left_boundary[:, 1], label="Left boundary")
    ax.plot(right_boundary[:, 0], right_boundary[:, 1], label="Right boundary")

    ax.plot(path.x, path.y, linewidth=2, label="Optimized path")
    ax.scatter(
        path.control_points[:, 0],
        path.control_points[:, 1],
        s=50,
        label="Control points",
        zorder=5,
    )

    ax.set_aspect("equal")
    ax.set_title(f"Baku T15-inspired segment: optimized path (v_in={v_in:.1f} m/s)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    plt.close(fig)


def plot_curvature_and_speed(path, q: np.ndarray, v_in: float, save_path: Path) -> None:
    v = np.sqrt(np.maximum(q, 1e-8))

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axes[0].plot(path.s_path, path.curvature)
    axes[0].set_ylabel("Curvature")
    axes[0].set_title(f"Curvature profile (v_in={v_in:.1f} m/s)")

    axes[1].plot(path.s_path, v)
    axes[1].set_ylabel("Speed")
    axes[1].set_xlabel("Path arc length")
    axes[1].set_title(f"Optimized speed profile (v_in={v_in:.1f} m/s)")

    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    plt.close(fig)


def plot_overlay(all_paths: list[dict], save_path: Path) -> None:
    """
    Overlay optimized paths for all entry speeds.
    """
    if not all_paths:
        return

    cfg = all_paths[0]["cfg"]
    centerline = cfg.centerline_xy
    heading = cfg.centerline_heading
    w = cfg.track_half_width

    normals = np.column_stack([-np.sin(heading), np.cos(heading)])
    left_boundary = centerline + w * normals
    right_boundary = centerline - w * normals

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(centerline[:, 0], centerline[:, 1], "--", label="Centerline")
    ax.plot(left_boundary[:, 0], left_boundary[:, 1], label="Left boundary")
    ax.plot(right_boundary[:, 0], right_boundary[:, 1], label="Right boundary")

    for item in all_paths:
        path = item["path"]
        v_in = item["v_in"]
        ax.plot(path.x, path.y, linewidth=2, label=f"v_in={v_in:.1f}")

    ax.set_aspect("equal")
    ax.set_title("Baku T15-inspired segment: optimal paths across entry speeds")
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    plt.close(fig)


def main():
    ensure_dirs()

    entry_speeds = [16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 26.0]

    rows = []
    all_paths = []

    previous_solution = None

    for v_in in entry_speeds:
        print("\n" + "=" * 72)
        print(f"Running case: v_in = {v_in:.1f} m/s")

        cfg = build_cfg_for_speed(v_in)
        builder = PathBuilder(cfg)

        if previous_solution is None:
            result = solve_single_level(cfg)
        else:
            x0 = warm_start_from_previous(previous_solution, cfg)
            result = solve_single_level(cfg, x0=x0)

        x_opt = result.x
        s_turn, s_apex, s_exit = x_opt[:3]
        q_opt = x_opt[3:]
        path = builder.build_path(s_turn, s_apex, s_exit)
        
        # Save per-case detailed path and speed profile
        case_tag = f"vin_{int(v_in)}"
        detail_df = pd.DataFrame(
            {
                "s_path": path.s_path,
                "x": path.x,
                "y": path.y,
                "curvature": path.curvature,
                "lateral_offset": path.lateral_offset,
                "q": q_opt,
                "v": np.sqrt(np.maximum(q_opt, 1e-8)),
            }
        )
        detail_df.to_csv(RUNS_DIR / f"{case_tag}_detail.csv", index=False)

        diagnostics = diagnose_constraints(x_opt, cfg, builder)

        q_min = float(np.min(q_opt))
        q_max = float(np.max(q_opt))
        v_min = float(np.sqrt(max(q_min, 1e-8)))
        v_max = float(np.sqrt(max(q_max, 1e-8)))

        print("Success:", result.success)
        print("Message:", result.message)
        print(f"Objective value: {result.fun:.6f}")
        print(f"s_turn = {s_turn:.4f}")
        print(f"s_apex = {s_apex:.4f}")
        print(f"s_exit = {s_exit:.4f}")
        print(f"v range = [{v_min:.4f}, {v_max:.4f}]")
        print("Constraint summary:")
        print(f"  Ordering:     {diagnostics['ordering']['max_violation']:.6f}")
        print(f"  Track-width:  {diagnostics['track_width']['max_violation']:.6f}")
        print(f"  Friction:     {diagnostics['friction']['max_violation']:.6f}")
        print(f"  Acceleration: {diagnostics['acceleration']['max_violation']:.6f}")
        print(f"  Braking:      {diagnostics['braking']['max_violation']:.6f}")
        print(f"  Entry-speed:  {diagnostics['entry_speed']['max_violation']:.6f}")
        print(f"  Bounds:       {diagnostics['bounds']['max_violation']:.6f}")

        # Save figures
        case_tag = f"vin_{int(v_in)}"
        plot_track_and_path(
            cfg,
            path,
            v_in,
            FIG_DIR / f"{case_tag}_path.png",
        )
        plot_curvature_and_speed(
            path,
            q_opt,
            v_in,
            FIG_DIR / f"{case_tag}_curvature_speed.png",
        )

        # Save row
        rows.append(
            {
                "v_in": v_in,
                "success": bool(result.success),
                "message": str(result.message),
                "objective_value": float(result.fun),
                "s_turn": float(s_turn),
                "s_apex": float(s_apex),
                "s_exit": float(s_exit),
                "q_min": q_min,
                "q_max": q_max,
                "v_min": v_min,
                "v_max": v_max,
                "path_length": float(path.path_length),
                "curvature_min": float(path.curvature.min()),
                "curvature_max": float(path.curvature.max()),
                "lat_offset_min": float(path.lateral_offset.min()),
                "lat_offset_max": float(path.lateral_offset.max()),
                "ordering_violation": diagnostics["ordering"]["max_violation"],
                "track_width_violation": diagnostics["track_width"]["max_violation"],
                "friction_violation": diagnostics["friction"]["max_violation"],
                "acceleration_violation": diagnostics["acceleration"]["max_violation"],
                "braking_violation": diagnostics["braking"]["max_violation"],
                "entry_speed_violation": diagnostics["entry_speed"]["max_violation"],
                "bounds_violation": diagnostics["bounds"]["max_violation"],
            }
        )

        all_paths.append(
            {
                "cfg": deepcopy(cfg),
                "path": path,
                "v_in": v_in,
            }
        )

        previous_solution = x_opt.copy()

    # Save summary CSV
    df = pd.DataFrame(rows)
    csv_path = RUNS_DIR / "baku_t15_speed_sweep_summary.csv"
    df.to_csv(csv_path, index=False)

    # Save overlay figure
    plot_overlay(all_paths, FIG_DIR / "baku_t15_speed_sweep_overlay.png")

    print("\n" + "=" * 72)
    print("Sweep finished.")
    print(f"Summary CSV saved to: {csv_path}")
    print(f"Figures saved to: {FIG_DIR}")


if __name__ == "__main__":
    main()