#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


EXPERIMENTS = {
    "subgoal_timing_2d": {
        "name": "URECA Subgoal Timing 2D",
        "mode": "single_k",
        "dim": 2,
        "start": (0.0, 0.0, 0.0),
        "subgoal": (1.0, 1.05, 0.0),
        "goal": (2.0, 1.0, 0.0),
        "obstacle_center": (1.0, 0.6, 0.0),
        "obstacle_radius": 0.20,
        "robot_radius": 0.10,
        "n_steps": 25,
    },
    "subgoal_timing_3d": {
        "name": "URECA Subgoal Timing 3D",
        "mode": "single_k",
        "dim": 3,
        "start": (0.0, 0.0, 0.0),
        "subgoal": (1.0, 1.05, 0.0),
        "goal": (2.0, 1.0, 0.0),
        "obstacle_center": (0.5, 0.6, 0.0),
        "obstacle_radius": 0.20,
        "robot_radius": 0.10,
        "n_steps": 25,
    },
    "penetration_failure_3d": {
        "name": "URECA Penetration Failure 3D",
        "mode": "penetration",
        "dim": 3,
        "start": (0.0, 0.0, 0.0),
        "subgoal": (1.0, 1.05, 0.0),
        "goal": (2.0, 1.0, 0.0),
        "obstacle_center": (0.5, 0.6, 0.0),
        "obstacle_radius": 0.20,
        "robot_radius": 0.10,
        "n_steps": 25,
        "default_k": 12,
    },
    "symmetry_failure_3d": {
        "name": "URECA Symmetry Failure 3D",
        "mode": "symmetry",
        "dim": 3,
        "start": (0.0, 0.0, 0.0),
        "subgoal": (1.0, 1.0, 0.0),
        "goal": (2.0, 1.0, 0.0),
        "obstacle_center": (0.5, 0.5, 0.0),
        "obstacle_radius": 0.20,
        "robot_radius": 0.10,
        "n_steps": 25,
        "default_k": 12,
    },
    "two_robot_two_subgoal": {
        "name": "URECA Two Robot Two Subgoal",
        "mode": "two_robot",
        "n_steps": 10,
        "r1_start": (0.0, 0.0, 0.0),
        "r1_subgoal": (1.0, 1.2, 0.0),
        "r1_goal": (2.0, 1.0, 0.0),
        "r2_start": (2.0, 0.0, 0.0),
        "r2_subgoal": (1.0, -0.2, 0.0),
        "r2_goal": (0.0, 1.0, 0.0),
        "robot_radius": 0.10,
    },
    "two_robot_two_subgoal_obstacle": {
        "name": "URECA Two Robot Two Subgoal Obstacle",
        "mode": "two_robot",
        "n_steps": 10,
        "r1_start": (0.0, 0.0, 0.0),
        "r1_subgoal": (1.0, 1.2, 0.0),
        "r1_goal": (2.0, 1.0, 0.0),
        "r2_start": (2.0, 0.0, 0.0),
        "r2_subgoal": (1.0, -0.2, 0.0),
        "r2_goal": (0.0, 1.0, 0.0),
        "obstacle_center": (1.0, 0.4, 0.0),
        "obstacle_radius": 0.25,
        "robot_radius": 0.10,
    },
    "success_obstacle_3d": {
        "name": "URECA Success Obstacle 3D",
        "mode": "single_k",
        "dim": 3,
        "start": (0.0, 0.0, 0.0),
        "subgoal": (1.0, 1.05, 0.0),
        "goal": (2.0, 1.0, 0.0),
        "obstacle_center": (0.5, 0.6, 0.0),
        "obstacle_radius": 0.20,
        "robot_radius": 0.10,
        "n_steps": 25,
        "default_k": 14,
    },
}


def read_summary(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def read_traj(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def f(row, key, default=float("nan")):
    value = row.get(key, "")
    if value in ("", "nan", "None", None):
        return default
    return float(value)


def ok(row):
    return row.get("trajopt_success", "0") in ("1", "true", "True")


def key_for_best(row):
    objective = f(row, "opt_path_objective", float("inf"))
    clearance = -f(row, "post_min_clearance_proxy", float("-inf"))
    return (objective, clearance)


def select_best_row(rows):
    candidates = [row for row in rows if row.get("trajectory_file") and ok(row)]
    if not candidates:
        candidates = [row for row in rows if row.get("trajectory_file")]
    if not candidates:
        return None
    return sorted(candidates, key=key_for_best)[0]


def interpolate(a, b, alpha):
    return tuple(a[i] + alpha * (b[i] - a[i]) for i in range(len(a)))


def reconstruct_single_seed(meta, row):
    n = int(meta["n_steps"])
    k = int(f(row, "K", meta.get("default_k", -1)))
    start = meta["start"]
    subgoal = meta["subgoal"]
    goal = meta["goal"]
    perturbation = f(row, "perturbation", 0.0)
    points = []
    for i in range(n):
        if i <= k:
            q = list(interpolate(start, subgoal, i / k))
        else:
            q = list(interpolate(subgoal, goal, (i - k) / (n - 1 - k)))
        if meta["mode"] == "symmetry" and 0 < i < n - 1:
            q[2] += perturbation
        points.append(q)
    return points


def reconstruct_two_robot_seed(meta, row):
    n = int(meta["n_steps"])
    k1 = int(f(row, "k1"))
    k2 = int(f(row, "k2"))
    points = []
    for i in range(n):
        if i <= k1:
            r1 = interpolate(meta["r1_start"], meta["r1_subgoal"], i / k1)
        else:
            r1 = interpolate(meta["r1_subgoal"], meta["r1_goal"], (i - k1) / (n - 1 - k1))
        if i <= k2:
            r2 = interpolate(meta["r2_start"], meta["r2_subgoal"], i / k2)
        else:
            r2 = interpolate(meta["r2_subgoal"], meta["r2_goal"], (i - k2) / (n - 1 - k2))
        points.append((*r1, *r2))
    return points


def draw_single_annotations(ax, meta, view="xy"):
    axes = (0, 1) if view == "xy" else (0, 2)
    start = meta["start"]
    subgoal = meta["subgoal"]
    goal = meta["goal"]
    ax.scatter(start[axes[0]], start[axes[1]], marker="s", s=80, color="tab:orange", label="start", zorder=5)
    ax.scatter(subgoal[axes[0]], subgoal[axes[1]], marker="*", s=180, color="tab:green", label="subgoal", zorder=5)
    ax.scatter(goal[axes[0]], goal[axes[1]], marker="D", s=100, color="tab:red", label="goal", zorder=5)

    if "obstacle_center" in meta and view in ("xy", "xz"):
        center = meta["obstacle_center"]
        obs = Circle((center[axes[0]], center[axes[1]]), meta["obstacle_radius"], fill=False, color="black", linewidth=2)
        ax.add_patch(obs)
        safe_radius = meta["obstacle_radius"] + meta["robot_radius"]
        safe = Circle((center[axes[0]], center[axes[1]]), safe_radius, fill=False, color="black", linestyle="--", linewidth=2)
        ax.add_patch(safe)
        ax.scatter(center[axes[0]], center[axes[1]], marker="x", s=120, color="tab:blue", label="obstacle center", zorder=5)


def draw_two_robot_annotations(ax, meta, view="xy"):
    axes = (0, 1) if view == "xy" else (0, 2)
    for prefix, color in (("r1", "tab:orange"), ("r2", "tab:purple")):
        start = meta[f"{prefix}_start"]
        subgoal = meta[f"{prefix}_subgoal"]
        goal = meta[f"{prefix}_goal"]
        ax.scatter(start[axes[0]], start[axes[1]], marker="s", s=70, color=color, label=f"{prefix} start", zorder=5)
        ax.scatter(subgoal[axes[0]], subgoal[axes[1]], marker="*", s=150, color=color, edgecolors="black",
                   label=f"{prefix} subgoal", zorder=5)
        ax.scatter(goal[axes[0]], goal[axes[1]], marker="D", s=90, color=color, edgecolors="black",
                   label=f"{prefix} goal", zorder=5)

    if "obstacle_center" in meta and view in ("xy", "xz"):
        center = meta["obstacle_center"]
        obs = Circle((center[axes[0]], center[axes[1]]), meta["obstacle_radius"], fill=False, color="black", linewidth=2)
        ax.add_patch(obs)
        safe_radius = meta["obstacle_radius"] + meta["robot_radius"]
        safe = Circle((center[axes[0]], center[axes[1]]), safe_radius, fill=False, color="black", linestyle="--", linewidth=2)
        ax.add_patch(safe)
        ax.scatter(center[axes[0]], center[axes[1]], marker="x", s=120, color="tab:blue", label="obstacle center", zorder=5)


def plot_all_single_trajectories(rows, result_dir, meta):
    plt.figure(figsize=(7, 6))
    ax = plt.gca()
    for row in rows:
        if not row.get("trajectory_file"):
            continue
        traj = read_traj(result_dir / "trajectories" / row["trajectory_file"])
        xs = [f(item, "x") for item in traj]
        ys = [f(item, "y") for item in traj]
        color = "tab:green" if ok(row) else "tab:red"
        alpha = 0.8 if ok(row) else 0.35
        ax.plot(xs, ys, color=color, alpha=alpha, linewidth=1.8)
    draw_single_annotations(ax, meta, "xy")
    ax.set_title(f'{meta["name"]}\nAll optimized XY trajectories')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.4)
    ax.axis("equal")
    ax.legend(fontsize=8, loc="best")
    plt.tight_layout()
    plt.savefig(result_dir / "figures" / "all_trajectories_xy.png", dpi=180)
    plt.close()


def plot_best_single(rows, result_dir, meta, view):
    best = select_best_row(rows)
    if best is None:
        return
    traj = read_traj(result_dir / "trajectories" / best["trajectory_file"])
    seed = reconstruct_single_seed(meta, best)
    ix, iy = (0, 1) if view == "xy" else (0, 2)
    traj_x = [f(item, "x") for item in traj]
    traj_y = [f(item, "y") if view == "xy" else f(item, "z") for item in traj]
    seed_x = [p[ix] for p in seed]
    seed_y = [p[iy] for p in seed]

    plt.figure(figsize=(7, 6))
    ax = plt.gca()
    ax.plot(seed_x, seed_y, "o--", color="tab:blue", linewidth=1.8, markersize=4, alpha=0.8,
            label=f'seed {best["case_id"]}')
    ax.plot(traj_x, traj_y, "o-", color="tab:orange", linewidth=2.2, markersize=5,
            label=f'optimized {best["case_id"]}')
    draw_single_annotations(ax, meta, view)
    ax.set_title(
        f'{meta["name"]}\nBest trajectory {view.upper()} projection, {best["case_id"]}, '
        f'clearance={f(best, "post_min_obstacle_clearance_proxy"):.4f}, '
        f'objective={f(best, "opt_path_objective"):.6f}'
    )
    ax.set_xlabel(view[0])
    ax.set_ylabel(view[1])
    ax.grid(True, alpha=0.4)
    if view == "xy":
        ax.axis("equal")
    ax.legend(fontsize=8, loc="best")
    plt.tight_layout()
    plt.savefig(result_dir / "figures" / f"best_trajectory_{view}.png", dpi=180)
    plt.close()


def plot_metric(rows, x_key, y_key, filename, xlabel, ylabel, title):
    xs = [f(row, x_key) for row in rows]
    ys = [f(row, y_key) for row in rows]
    mask = [(not math.isnan(x) and not math.isnan(y)) for x, y in zip(xs, ys)]
    xs = [x for x, keep in zip(xs, mask) if keep]
    ys = [y for y, keep in zip(ys, mask) if keep]
    plt.figure(figsize=(7, 4))
    plt.plot(xs, ys, marker="o")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    plt.savefig(filename, dpi=180)
    plt.close()


def plot_success(rows, x_key, filename, xlabel, title):
    xs = [f(row, x_key) for row in rows]
    ys = [1 if ok(row) else 0 for row in rows]
    plt.figure(figsize=(7, 4))
    plt.scatter(xs, ys, c=["tab:green" if y else "tab:red" for y in ys], s=55)
    plt.yticks([0, 1], ["fail", "success"])
    plt.xlabel(xlabel)
    plt.ylabel("trajopt_success")
    plt.title(title)
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    plt.savefig(filename, dpi=180)
    plt.close()


def plot_categorical(rows, label_fn, value_key, filename, ylabel, title):
    xs = list(range(len(rows)))
    labels = [label_fn(row, i) for i, row in enumerate(rows)]
    ys = [f(row, value_key) for row in rows]
    plt.figure(figsize=(8, 4))
    plt.plot(xs, ys, marker="o")
    plt.xticks(xs, labels, rotation=20, ha="right")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    plt.savefig(filename, dpi=180)
    plt.close()


def plot_categorical_success(rows, label_fn, filename, title):
    xs = list(range(len(rows)))
    labels = [label_fn(row, i) for i, row in enumerate(rows)]
    ys = [1 if ok(row) else 0 for row in rows]
    plt.figure(figsize=(8, 4))
    plt.scatter(xs, ys, c=["tab:green" if y else "tab:red" for y in ys], s=60)
    plt.xticks(xs, labels, rotation=20, ha="right")
    plt.yticks([0, 1], ["fail", "success"])
    plt.ylabel("trajopt_success")
    plt.title(title)
    plt.grid(True, alpha=0.35)
    plt.tight_layout()
    plt.savefig(filename, dpi=180)
    plt.close()


def plot_heatmap(rows, value_key, filename, title):
    k1s = sorted({int(f(row, "k1")) for row in rows if not math.isnan(f(row, "k1")) and int(f(row, "k1")) >= 0})
    k2s = sorted({int(f(row, "k2")) for row in rows if not math.isnan(f(row, "k2")) and int(f(row, "k2")) >= 0})
    if not k1s or not k2s:
        return
    grid = [[float("nan") for _ in k2s] for _ in k1s]
    for row in rows:
        k1 = int(f(row, "k1"))
        k2 = int(f(row, "k2"))
        value = f(row, value_key)
        grid[k1s.index(k1)][k2s.index(k2)] = value
    plt.figure(figsize=(6, 5))
    plt.imshow(grid, origin="lower", aspect="auto")
    plt.xticks(range(len(k2s)), k2s)
    plt.yticks(range(len(k1s)), k1s)
    plt.xlabel("k2")
    plt.ylabel("k1")
    plt.title(title)
    plt.colorbar()
    plt.tight_layout()
    plt.savefig(filename, dpi=180)
    plt.close()


def plot_two_robot_best(rows, result_dir, meta, view):
    best = select_best_row(rows)
    if best is None:
        return
    traj = read_traj(result_dir / "trajectories" / best["trajectory_file"])
    seed = reconstruct_two_robot_seed(meta, best)
    ix, iy = (0, 1) if view == "xy" else (0, 2)
    sx1 = [p[ix] for p in seed]
    sy1 = [p[iy] for p in seed]
    sx2 = [p[ix + 3] for p in seed]
    sy2 = [p[iy + 3] for p in seed]
    ox1 = [f(item, "r1_x") for item in traj]
    oy1 = [f(item, "r1_y") if view == "xy" else f(item, "r1_z") for item in traj]
    ox2 = [f(item, "r2_x") for item in traj]
    oy2 = [f(item, "r2_y") if view == "xy" else f(item, "r2_z") for item in traj]

    plt.figure(figsize=(7, 6))
    ax = plt.gca()
    ax.plot(sx1, sy1, "o--", color="tab:blue", alpha=0.75, label="robot1 seed")
    ax.plot(ox1, oy1, "o-", color="tab:orange", label="robot1 optimized")
    ax.plot(sx2, sy2, "o--", color="tab:cyan", alpha=0.75, label="robot2 seed")
    ax.plot(ox2, oy2, "o-", color="tab:purple", label="robot2 optimized")
    draw_two_robot_annotations(ax, meta, view)
    ax.set_title(
        f'{meta["name"]}\nBest trajectory {view.upper()} projection, {best["case_id"]}, '
        f'clearance={f(best, "post_min_clearance_proxy"):.4f}, '
        f'objective={f(best, "opt_path_objective"):.6f}'
    )
    ax.set_xlabel(view[0])
    ax.set_ylabel(view[1])
    ax.grid(True, alpha=0.4)
    if view == "xy":
        ax.axis("equal")
    ax.legend(fontsize=8, loc="best")
    plt.tight_layout()
    plt.savefig(result_dir / "figures" / f"best_trajectory_{view}.png", dpi=180)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir")
    args = parser.parse_args()

    result_dir = Path(args.result_dir)
    key = result_dir.name
    if key not in EXPERIMENTS:
        raise SystemExit(f"Unsupported result directory: {key}")

    meta = EXPERIMENTS[key]
    rows = read_summary(result_dir / "summary.csv")
    (result_dir / "figures").mkdir(parents=True, exist_ok=True)

    if meta["mode"] == "two_robot":
        success_rows = [dict(row, success_value=("1" if ok(row) else "0")) for row in rows]
        plot_heatmap([dict(row, success_value=("1" if ok(row) else "0")) for row in rows],
                     "success_value",
                     result_dir / "figures" / "heatmap_success_k1_k2.png",
                     f'{meta["name"]}\nSuccess heatmap')
        plot_heatmap(rows,
                     "post_path_length",
                     result_dir / "figures" / "heatmap_path_length_k1_k2.png",
                     f'{meta["name"]}\nPath length heatmap')
        plot_heatmap(rows,
                     "post_min_inter_robot_clearance_proxy",
                     result_dir / "figures" / "heatmap_inter_robot_clearance_k1_k2.png",
                     f'{meta["name"]}\nInter-robot clearance heatmap')
        plot_heatmap(rows,
                     "opt_path_objective",
                     result_dir / "figures" / "heatmap_objective_k1_k2.png",
                     f'{meta["name"]}\nObjective heatmap')
        plot_two_robot_best(rows, result_dir, meta, "xy")
        plot_two_robot_best(rows, result_dir, meta, "xz")
        return

    if meta["mode"] == "single_k":
        plot_all_single_trajectories(rows, result_dir, meta)
        plot_best_single(rows, result_dir, meta, "xy")
        plot_best_single(rows, result_dir, meta, "xz")
        plot_metric(rows,
                    "K",
                    "opt_path_objective",
                    result_dir / "figures" / "k_vs_objective.png",
                    "K",
                    "opt_path_objective",
                    f'{meta["name"]}\nObjective vs K')
        plot_metric(rows,
                    "K",
                    "post_path_length",
                    result_dir / "figures" / "k_vs_path_length.png",
                    "K",
                    "post_path_length",
                    f'{meta["name"]}\nPath length vs K')
        plot_metric(rows,
                    "K",
                    "post_min_obstacle_clearance_proxy",
                    result_dir / "figures" / "k_vs_clearance.png",
                    "K",
                    "post_min_obstacle_clearance_proxy",
                    f'{meta["name"]}\nClearance vs K')
        plot_metric(rows,
                    "K",
                    "solve_time_ms",
                    result_dir / "figures" / "k_vs_solve_time.png",
                    "K",
                    "solve_time_ms",
                    f'{meta["name"]}\nSolve time vs K')
        plot_success(rows,
                     "K",
                     result_dir / "figures" / "k_vs_success.png",
                     "K",
                     f'{meta["name"]}\nSuccess vs K')
        return

    if meta["mode"] == "symmetry":
        plot_all_single_trajectories(rows, result_dir, meta)
        plot_best_single(rows, result_dir, meta, "xy")
        plot_best_single(rows, result_dir, meta, "xz")
        plot_metric(rows,
                    "perturbation",
                    "opt_path_objective",
                    result_dir / "figures" / "perturbation_vs_objective.png",
                    "perturbation",
                    "opt_path_objective",
                    f'{meta["name"]}\nObjective vs perturbation')
        plot_metric(rows,
                    "perturbation",
                    "post_min_obstacle_clearance_proxy",
                    result_dir / "figures" / "perturbation_vs_clearance.png",
                    "perturbation",
                    "post_min_obstacle_clearance_proxy",
                    f'{meta["name"]}\nClearance vs perturbation')
        plot_success(rows,
                     "perturbation",
                     result_dir / "figures" / "perturbation_vs_success.png",
                     "perturbation",
                     f'{meta["name"]}\nSuccess vs perturbation')
        (result_dir / "figures" / "symmetry_trajectories_xy.png").write_bytes(
            (result_dir / "figures" / "all_trajectories_xy.png").read_bytes()
        )
        return

    plot_all_single_trajectories(rows, result_dir, meta)
    plot_best_single(rows, result_dir, meta, "xy")
    plot_best_single(rows, result_dir, meta, "xz")
    plot_categorical_success(
        rows,
        lambda row, i: row.get("penetration_label") or row.get("case_id", str(i)),
        result_dir / "figures" / "penetration_vs_success.png",
        f'{meta["name"]}\nSuccess vs penetration case',
    )
    plot_categorical(
        rows,
        lambda row, i: row.get("penetration_label") or row.get("case_id", str(i)),
        "post_min_obstacle_clearance_proxy",
        result_dir / "figures" / "penetration_vs_clearance.png",
        "post_min_obstacle_clearance_proxy",
        f'{meta["name"]}\nClearance vs penetration case',
    )
    plot_categorical(
        rows,
        lambda row, i: row.get("penetration_label") or row.get("case_id", str(i)),
        "opt_path_objective",
        result_dir / "figures" / "penetration_vs_objective.png",
        "opt_path_objective",
        f'{meta["name"]}\nObjective vs penetration case',
    )
    (result_dir / "figures" / "penetration_trajectories_xy.png").write_bytes(
        (result_dir / "figures" / "all_trajectories_xy.png").read_bytes()
    )


if __name__ == "__main__":
    main()
