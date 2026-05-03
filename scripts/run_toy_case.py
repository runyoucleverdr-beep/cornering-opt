import numpy as np
import matplotlib.pyplot as plt

from cornering.utils.baku_t15 import build_baku_t15_config
from cornering.geometry.path_builder import PathBuilder
from cornering.optimization.solver import solve_single_level
from cornering.optimization.constraints import diagnose_constraints


def plot_track_and_path(cfg, path):
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
    ax.set_title("Baku Turn 15-inspired segment: optimized racing line")
    ax.legend()
    plt.show()


def plot_curvature_and_speed(path, q):
    v = np.sqrt(np.maximum(q, 1e-8))

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axes[0].plot(path.s_path, path.curvature)
    axes[0].set_ylabel("Curvature")
    axes[0].set_title("Baku Turn 15-inspired segment: curvature profile")

    axes[1].plot(path.s_path, v)
    axes[1].set_ylabel("Speed")
    axes[1].set_xlabel("Path arc length")
    axes[1].set_title("Baku Turn 15-inspired segment: optimized speed profile")

    plt.tight_layout()
    plt.show()


def main():
    cfg = build_baku_t15_config()
    print("s_turn_bounds:", cfg.s_turn_bounds)
    print("total centerline length:", cfg.centerline_s[-1])
    builder = PathBuilder(cfg)

    # Stage 1
    result1 = solve_single_level(cfg)

    # Stage 2 warm start from projected stage-1 solution
    from cornering.optimization.solver import project_to_basic_feasible_guess

    x0_stage2 = project_to_basic_feasible_guess(result1.x, cfg)
    result = solve_single_level(cfg, x0=x0_stage2)

    print("\n=== Optimization Result ===")
    print("Success:", result.success)
    print("Message:", result.message)
    print("Objective value:", result.fun)

    x_opt = result.x
    s_turn, s_apex, s_exit = x_opt[:3]
    q_opt = x_opt[3:]

    print(f"s_turn = {s_turn:.4f}")
    print(f"s_apex = {s_apex:.4f}")
    print(f"s_exit = {s_exit:.4f}")
    print(f"q range = [{q_opt.min():.4f}, {q_opt.max():.4f}]")

    path = builder.build_path(s_turn, s_apex, s_exit)
    diagnostics = diagnose_constraints(x_opt, cfg, builder)

    print("\n=== Constraint Diagnostics ===")
    print(f"Ordering max violation:     {diagnostics['ordering']['max_violation']:.6f}")
    print(f"Track-width max violation:  {diagnostics['track_width']['max_violation']:.6f}")
    print(f"Friction max violation:     {diagnostics['friction']['max_violation']:.6f}")
    print(f"Acceleration max violation: {diagnostics['acceleration']['max_violation']:.6f}")
    print(f"Braking max violation:      {diagnostics['braking']['max_violation']:.6f}")
    print(f"Entry-speed violation:      {diagnostics['entry_speed']['max_violation']:.6f}")

    print(f"Path length: {path.path_length:.3f}")
    print(
        f"Curvature range: [{path.curvature.min():.4f}, {path.curvature.max():.4f}]"
    )
    print(
        f"Lateral offset range: [{path.lateral_offset.min():.4f}, {path.lateral_offset.max():.4f}]"
    )
    print(f"Bounds max violation:       {diagnostics['bounds']['max_violation']:.6f}")
    print("Bounds details:")
    for k, v in diagnostics["bounds"]["details"].items():
        print(f"  {k}: {v:.6f}")

    plot_track_and_path(cfg, path)
    plot_curvature_and_speed(path, q_opt)


if __name__ == "__main__":
    main()