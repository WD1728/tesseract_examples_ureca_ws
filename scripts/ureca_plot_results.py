#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(summary_path: Path):
    with summary_path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def to_float(row, key, default=0.0):
    value = row.get(key, "")
    if value in ("", "nan", "None", None):
        return default
    return float(value)


def to_int(row, key, default=0):
    value = row.get(key, "")
    if value in ("", "None", None):
        return default
    return int(float(value))


def read_trajectory(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def plot_metric(rows, x_key, y_key, output, title, xlabel, ylabel):
    xs = [to_float(row, x_key) for row in rows]
    ys = [to_float(row, y_key) for row in rows]
    colors = ["tab:green" if row.get("success", "0") in ("1", "true", "True") else "tab:red" for row in rows]
    plt.figure(figsize=(7, 4))
    plt.scatter(xs, ys, c=colors)
    plt.plot(xs, ys, alpha=0.4)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    plt.close()


def plot_trajectories(rows, summary_dir, output, title, x_col="x", y_col="y", best_only=False):
    plt.figure(figsize=(6, 6))
    successful = [row for row in rows if row.get("trajectory_file")]
    if not successful:
        plt.title(title + " (no successful trajectories)")
        plt.tight_layout()
        plt.savefig(output, dpi=160)
        plt.close()
        return

    if best_only:
        successful = sorted(successful, key=lambda row: to_float(row, "objective", 1e18))[:1]

    for row in successful:
        traj = read_trajectory(summary_dir / row["trajectory_file"])
        xs = [to_float(item, x_col) for item in traj]
        ys = [to_float(item, y_col) for item in traj]
        plt.plot(xs, ys, alpha=0.7, label=row["case_id"])

    plt.title(title)
    plt.xlabel(x_col)
    plt.ylabel(y_col)
    if len(successful) <= 10:
        plt.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    plt.close()


def plot_success(rows, x_key, output, title):
    xs = [to_float(row, x_key) for row in rows]
    ys = [1 if row.get("success", "0") in ("1", "true", "True") else 0 for row in rows]
    plt.figure(figsize=(7, 4))
    plt.scatter(xs, ys, c=["tab:green" if y else "tab:red" for y in ys])
    plt.yticks([0, 1], ["fail", "success"])
    plt.title(title)
    plt.xlabel(x_key)
    plt.ylabel("success")
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    plt.close()


def save_heatmap(rows, output, value_key, title):
    radii = sorted({to_float(row, "obstacle_radius") for row in rows})
    margins = sorted({to_float(row, "safe_radius") - to_float(row, "obstacle_radius") - 0.10 for row in rows})
    if not radii or not margins:
        return
    grid = [[0.0 for _ in margins] for _ in radii]
    for row in rows:
        r = to_float(row, "obstacle_radius")
        m = to_float(row, "safe_radius") - to_float(row, "obstacle_radius") - 0.10
        grid[radii.index(r)][margins.index(m)] = to_float(row, value_key)

    plt.figure(figsize=(6, 4))
    plt.imshow(grid, origin="lower", aspect="auto")
    plt.xticks(range(len(margins)), [f"{m:.2f}" for m in margins])
    plt.yticks(range(len(radii)), [f"{r:.2f}" for r in radii])
    plt.xlabel("safe_margin")
    plt.ylabel("obstacle_radius")
    plt.title(title)
    plt.colorbar()
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    plt.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", required=True)
    args = parser.parse_args()

    summary_path = Path(args.summary)
    rows = read_rows(summary_path)
    summary_dir = summary_path.parent
    experiment_id = rows[0]["experiment_id"] if rows else summary_path.parent.name

    if experiment_id in {"E1_2d_k_sweep", "E2_3d_k_sweep"}:
        plot_trajectories(rows, summary_dir, summary_dir / "all_trajectories_xy.png", "All trajectories", "x", "y")
        plot_trajectories(rows, summary_dir, summary_dir / "best_trajectory_xy.png", "Best trajectory", "x", "y", best_only=True)
        if experiment_id == "E2_3d_k_sweep":
            plot_trajectories(rows, summary_dir, summary_dir / "all_trajectories_xz.png", "All trajectories XZ", "x", "z")
            plot_trajectories(rows, summary_dir, summary_dir / "best_trajectory_xz.png", "Best trajectory XZ", "x", "z", best_only=True)
        plot_metric(rows, "K", "objective", summary_dir / "objective_vs_k.png", "Objective vs K", "K", "objective")
        plot_metric(rows, "K", "opt_min_clearance", summary_dir / "clearance_vs_k.png", "Clearance vs K", "K", "clearance")
        plot_metric(rows, "K", "solve_time_ms", summary_dir / "solve_time_vs_k.png", "Solve time vs K", "K", "ms")
        plot_success(rows, "K", summary_dir / "success_vs_k.png", "Success vs K")
    elif experiment_id == "E3_no_subgoal_vs_subgoal":
        plot_trajectories(rows, summary_dir, summary_dir / "no_subgoal_vs_subgoal_xy.png", "No-subgoal vs subgoal", "x", "y")
        plot_metric(rows, "K", "objective", summary_dir / "objective_comparison.png", "Objective comparison", "K", "objective")
        plot_metric(rows, "K", "opt_min_clearance", summary_dir / "clearance_comparison.png", "Clearance comparison", "K", "clearance")
    elif experiment_id == "E4_seed_penetration_depth":
        plot_metric(rows, "seed_min_clearance", "opt_min_clearance", summary_dir / "seed_penetration_vs_opt_clearance.png", "Seed penetration vs optimized clearance", "seed_min_clearance", "opt_min_clearance")
        plot_success(rows, "seed_min_clearance", summary_dir / "seed_penetration_vs_success.png", "Seed penetration vs success")
        plot_metric(rows, "seed_min_clearance", "objective", summary_dir / "seed_penetration_vs_objective.png", "Seed penetration vs objective", "seed_min_clearance", "objective")
        plot_trajectories([row for row in rows if row.get("success", "0") not in ("1", "true", "True")], summary_dir, summary_dir / "failure_cases_xy.png", "Failure cases", "x", "y")
    elif experiment_id == "E5_symmetry_breaking":
        plot_trajectories(rows, summary_dir, summary_dir / "symmetry_trajectories_xy.png", "Symmetry trajectories", "x", "y")
        plot_metric(rows, "perturbation", "objective", summary_dir / "perturbation_vs_objective.png", "Perturbation vs objective", "perturbation", "objective")
        plot_metric(rows, "perturbation", "opt_min_clearance", summary_dir / "perturbation_vs_clearance.png", "Perturbation vs clearance", "perturbation", "clearance")
        plot_success(rows, "perturbation", summary_dir / "perturbation_vs_success.png", "Perturbation vs success")
    elif experiment_id == "E6_obstacle_difficulty_sweep":
        success_rows = [dict(row, objective=("1" if row.get("success", "0") in ("1", "true", "True") else "0")) for row in rows]
        save_heatmap(success_rows, summary_dir / "success_heatmap_radius_margin.png", "objective", "Success heatmap")
        save_heatmap(rows, summary_dir / "clearance_heatmap_radius_margin.png", "opt_min_clearance", "Clearance heatmap")
        save_heatmap(rows, summary_dir / "objective_heatmap_radius_margin.png", "objective", "Objective heatmap")
    elif experiment_id == "E7_subgoal_location_sweep":
        plot_trajectories(rows, summary_dir, summary_dir / "subgoal_location_trajectories_xy.png", "Subgoal location trajectories", "x", "y")
        plot_metric(rows, "subgoal_y", "objective", summary_dir / "subgoal_location_vs_objective.png", "Subgoal location vs objective", "subgoal_y", "objective")
        plot_metric(rows, "subgoal_y", "opt_min_clearance", summary_dir / "subgoal_location_vs_clearance.png", "Subgoal location vs clearance", "subgoal_y", "clearance")
    elif experiment_id == "E8_blackbox_wrapper_demo":
        plot_trajectories(rows, summary_dir, summary_dir / "blackbox_demo_trajectory_xy.png", "Black-box demo trajectory", "x", "y", best_only=True)
    else:
        plot_trajectories(rows, summary_dir, summary_dir / "all_trajectories_xy.png", "Trajectories", "x", "y")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
