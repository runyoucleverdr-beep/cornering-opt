from __future__ import annotations

from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from cornering.utils.baku_t15 import build_baku_t15_config
from cornering.geometry.path_builder import PathBuilder


CSV_PATH = Path("results/runs/baku_t15_speed_sweep_summary.csv")
FIG_DIR = Path("results/figures/baku_t15_speed_sweep_analysis")


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def load_results() -> tuple[pd.DataFrame, pd.DataFrame]:
    df = pd.read_csv(CSV_PATH)
    df = df.sort_values("v_in").reset_index(drop=True)

    # Normalize success column if needed
    if df["success"].dtype == object:
        df["success"] = df["success"].astype(str).str.lower().isin(["true", "1", "yes"])

    df_success = df[df["success"]].copy().reset_index(drop=True)
    return df, df_success


def plot_objective(df_all: pd.DataFrame, df_success: pd.DataFrame) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(df_all["v_in"], df_all["objective_value"], "o--", label="All runs")
    if not df_success.empty:
        ax.plot(df_success["v_in"], df_success["objective_value"], "o-", linewidth=2, label="Successful runs")

    ax.set_xlabel("Entry speed v_in (m/s)")
    ax.set_ylabel("Objective value")
    ax.set_title("Objective value vs entry speed")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "objective_vs_entry_speed.png", dpi=200)
    plt.close(fig)


def plot_line_parameters(df_success: pd.DataFrame) -> None:
    if df_success.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(df_success["v_in"], df_success["s_turn"], "o-", label="s_turn")
    ax.plot(df_success["v_in"], df_success["s_apex"], "o-", label="s_apex")
    ax.plot(df_success["v_in"], df_success["s_exit"], "o-", label="s_exit")

    ax.set_xlabel("Entry speed v_in (m/s)")
    ax.set_ylabel("Optimal geometric parameter")
    ax.set_title("Optimal racing-line parameters vs entry speed")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "line_parameters_vs_entry_speed.png", dpi=200)
    plt.close(fig)


def plot_speed_range(df_success: pd.DataFrame) -> None:
    if df_success.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(df_success["v_in"], df_success["v_min"], "o-", label="v_min")
    ax.plot(df_success["v_in"], df_success["v_max"], "o-", label="v_max")

    ax.set_xlabel("Entry speed v_in (m/s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed range vs entry speed")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "speed_range_vs_entry_speed.png", dpi=200)
    plt.close(fig)

def plot_vmin_vs_entry_speed(df_success: pd.DataFrame) -> None:
    if df_success.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(df_success["v_in"], df_success["v_min"], "o-", linewidth=2)
    ax.set_xlabel("Entry speed v_in (m/s)")
    ax.set_ylabel("Minimum speed through corner (m/s)")
    ax.set_title("Minimum corner speed vs entry speed")

    fig.tight_layout()
    fig.savefig(FIG_DIR / "vmin_vs_entry_speed.png", dpi=200)
    plt.close(fig)


def plot_speed_profile_overlay(df_success: pd.DataFrame) -> None:
    if df_success.empty:
        return

    fig, ax = plt.subplots(figsize=(9, 5))

    for _, row in df_success.iterrows():
        v_in = int(row["v_in"])
        detail_path = Path("results/runs") / f"vin_{v_in}_detail.csv"

        if not detail_path.exists():
            continue

        detail_df = pd.read_csv(detail_path)
        ax.plot(
            detail_df["s_path"],
            detail_df["v"],
            linewidth=2,
            label=f"v_in={v_in}",
        )

    ax.set_xlabel("Path arc length")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Optimized speed profiles across successful entry speeds")
    ax.legend()

    fig.tight_layout()
    fig.savefig(FIG_DIR / "speed_profile_overlay.png", dpi=200)
    plt.close(fig)

def plot_constraint_violations(df_all: pd.DataFrame) -> None:
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(df_all["v_in"], df_all["track_width_violation"], "o-", label="track_width")
    ax.plot(df_all["v_in"], df_all["friction_violation"], "o-", label="friction")
    ax.plot(df_all["v_in"], df_all["acceleration_violation"], "o-", label="acceleration")
    ax.plot(df_all["v_in"], df_all["braking_violation"], "o-", label="braking")
    ax.plot(df_all["v_in"], df_all["entry_speed_violation"], "o-", label="entry_speed")
    ax.plot(df_all["v_in"], df_all["bounds_violation"], "o-", label="bounds")

    ax.set_xlabel("Entry speed v_in (m/s)")
    ax.set_ylabel("Max violation")
    ax.set_title("Constraint violations vs entry speed")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "constraint_violations_vs_entry_speed.png", dpi=200)
    plt.close(fig)


def plot_successful_path_overlay(df_success: pd.DataFrame) -> None:
    if df_success.empty:
        return

    cfg = build_baku_t15_config()
    builder = PathBuilder(cfg)

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

    for _, row in df_success.iterrows():
        path = builder.build_path(row["s_turn"], row["s_apex"], row["s_exit"])
        ax.plot(path.x, path.y, linewidth=2, label=f"v_in={row['v_in']:.0f}")

    ax.set_aspect("equal")
    ax.set_title("Successful optimal paths across entry speeds")
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "successful_path_overlay.png", dpi=200)
    plt.close(fig)


def save_clean_summary(df_all: pd.DataFrame, df_success: pd.DataFrame) -> None:
    df_all.to_csv(FIG_DIR / "all_runs_sorted.csv", index=False)
    df_success.to_csv(FIG_DIR / "successful_runs_only.csv", index=False)


def main() -> None:
    ensure_dirs()

    if not CSV_PATH.exists():
        raise FileNotFoundError(f"Summary CSV not found: {CSV_PATH}")

    df_all, df_success = load_results()

    print("Loaded summary file:", CSV_PATH)
    print(f"Total runs: {len(df_all)}")
    print(f"Successful runs: {len(df_success)}")

    if not df_success.empty:
        print("\nSuccessful cases:")
        print(df_success[["v_in", "objective_value", "s_turn", "s_apex", "s_exit", "v_min", "v_max"]])

    plot_objective(df_all, df_success)
    plot_line_parameters(df_success)
    plot_speed_range(df_success)
    plot_constraint_violations(df_all)
    plot_successful_path_overlay(df_success)
    plot_speed_profile_overlay(df_success)
    save_clean_summary(df_all, df_success)

    print("\nAnalysis figures saved to:", FIG_DIR)


if __name__ == "__main__":
    main()