#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_summary(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def read_traj(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def f(row, key, default=0.0):
    value = row.get(key, "")
    if value in ("", "nan", "None", None):
        return default
    return float(value)


def ok(row):
    return row.get("trajopt_success", "0") in ("1", "true", "True")


def plot_single_trajectories(rows, result_dir, figure_name, x_key="x", y_key="y", best_only=False):
    candidates = [row for row in rows if row.get("trajectory_file")]
    if best_only and candidates:
        candidates = sorted(candidates, key=lambda r: f(r, "post_path_length", 1e18))[:1]
    plt.figure(figsize=(6, 6))
    for row in candidates:
        traj = read_traj(result_dir / "trajectories" / row["trajectory_file"])
        xs = [f(item, x_key) for item in traj]
        ys = [f(item, y_key) for item in traj]
        plt.plot(xs, ys, alpha=0.75, label=row["case_id"])
    if candidates and len(candidates) <= 10:
        plt.legend(fontsize=7)
    plt.xlabel(x_key)
    plt.ylabel(y_key)
    plt.tight_layout()
    plt.savefig(result_dir / "figures" / figure_name, dpi=160)
    plt.close()


def plot_k_metric(rows, y_key, filename, ylabel):
    xs = [f(row, "K") for row in rows]
    ys = [f(row, y_key) for row in rows]
    plt.figure(figsize=(7, 4))
    plt.plot(xs, ys, marker="o")
    plt.xlabel("K")
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    plt.close()


def plot_success(rows, x_key, filename):
    xs = [f(row, x_key) for row in rows]
    ys = [1 if ok(row) else 0 for row in rows]
    plt.figure(figsize=(7, 4))
    plt.scatter(xs, ys, c=["tab:green" if y else "tab:red" for y in ys])
    plt.yticks([0, 1], ["fail", "success"])
    plt.xlabel(x_key)
    plt.ylabel("success")
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    plt.close()


def plot_categorical(rows, value_key, filename, ylabel):
    xs = list(range(len(rows)))
    labels = [row.get("penetration_label") or row.get("case_id", str(i)) for i, row in enumerate(rows)]
    ys = [f(row, value_key) for row in rows]
    plt.figure(figsize=(8, 4))
    plt.plot(xs, ys, marker="o")
    plt.xticks(xs, labels, rotation=20, ha="right")
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    plt.close()


def plot_heatmap(rows, value_key, filename):
    k1s = sorted({int(f(row, "k1")) for row in rows if int(f(row, "k1", -1)) >= 0})
    k2s = sorted({int(f(row, "k2")) for row in rows if int(f(row, "k2", -1)) >= 0})
    if not k1s or not k2s:
        return
    grid = [[0.0 for _ in k2s] for _ in k1s]
    for row in rows:
        k1 = int(f(row, "k1"))
        k2 = int(f(row, "k2"))
        grid[k1s.index(k1)][k2s.index(k2)] = f(row, value_key)
    plt.figure(figsize=(6, 5))
    plt.imshow(grid, origin="lower", aspect="auto")
    plt.xticks(range(len(k2s)), k2s)
    plt.yticks(range(len(k1s)), k1s)
    plt.xlabel("k2")
    plt.ylabel("k1")
    plt.colorbar()
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    plt.close()


def plot_two_robot_best(rows, result_dir, filename, x_key, y_key):
    candidates = [row for row in rows if row.get("trajectory_file")]
    if not candidates:
        return
    best = sorted(candidates, key=lambda r: f(r, "post_path_length", 1e18))[0]
    traj = read_traj(result_dir / "trajectories" / best["trajectory_file"])
    plt.figure(figsize=(6, 6))
    plt.plot([f(item, f"r1_{x_key}") for item in traj], [f(item, f"r1_{y_key}") for item in traj], label="robot1")
    plt.plot([f(item, f"r2_{x_key}") for item in traj], [f(item, f"r2_{y_key}") for item in traj], label="robot2")
    plt.legend()
    plt.xlabel(x_key)
    plt.ylabel(y_key)
    plt.tight_layout()
    plt.savefig(result_dir / "figures" / filename, dpi=160)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir")
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    summary_path = result_dir / "summary.csv"
    rows = read_summary(summary_path)
    (result_dir / "figures").mkdir(parents=True, exist_ok=True)

    has_two_robot = any(int(f(row, "k1", -1)) >= 0 for row in rows)
    has_perturbation = any(abs(f(row, "perturbation", 0.0)) > 0.0 or row.get("case_id") == "perturb_0.00" for row in rows)
    has_penetration = any(row.get("penetration_label") for row in rows)
    has_k = any(int(f(row, "K", -1)) >= 0 for row in rows)

    if has_two_robot:
        plot_heatmap([dict(row, post_path_length=("1" if ok(row) else "0")) for row in rows], "post_path_length", result_dir / "figures" / "heatmap_success_k1_k2.png")
        plot_heatmap(rows, "post_path_length", result_dir / "figures" / "heatmap_path_length_k1_k2.png")
        plot_heatmap(rows, "post_min_inter_robot_clearance_proxy", result_dir / "figures" / "heatmap_inter_robot_clearance_k1_k2.png")
        plot_two_robot_best(rows, result_dir, "best_trajectory_xy.png", "x", "y")
        plot_two_robot_best(rows, result_dir, "best_trajectory_xz.png", "x", "z")
    elif has_penetration:
        plot_categorical([{**row, "post_path_length": "1" if ok(row) else "0"} for row in rows], "post_path_length", result_dir / "figures" / "penetration_vs_success.png", "success")
        plot_categorical(rows, "post_min_obstacle_clearance_proxy", result_dir / "figures" / "penetration_vs_clearance.png", "post_min_obstacle_clearance_proxy")
        plot_single_trajectories(rows, result_dir, "penetration_trajectories_xy.png")
    elif has_perturbation:
        plot_success(rows, "perturbation", result_dir / "figures" / "perturbation_vs_success.png")
        plot_k_metric(rows, "post_min_obstacle_clearance_proxy", result_dir / "figures" / "perturbation_vs_clearance.png", "post_min_obstacle_clearance_proxy")
        plot_single_trajectories(rows, result_dir, "symmetry_trajectories_xy.png")
    elif has_k:
        plot_single_trajectories(rows, result_dir, "all_trajectories_xy.png")
        plot_single_trajectories(rows, result_dir, "best_trajectory_xy.png", best_only=True)
        if any((result_dir / "trajectories" / row.get("trajectory_file", "")).exists() for row in rows if row.get("trajectory_file")):
            plot_single_trajectories(rows, result_dir, "best_trajectory_xz.png", x_key="x", y_key="z", best_only=True)
        plot_k_metric(rows, "post_path_length", result_dir / "figures" / "k_vs_path_length.png", "post_path_length")
        plot_k_metric(rows, "post_min_obstacle_clearance_proxy", result_dir / "figures" / "k_vs_clearance.png", "post_min_obstacle_clearance_proxy")
        plot_k_metric(rows, "solve_time_ms", result_dir / "figures" / "k_vs_solve_time.png", "solve_time_ms")
        plot_success(rows, "K", result_dir / "figures" / "k_vs_success.png")


if __name__ == "__main__":
    main()
