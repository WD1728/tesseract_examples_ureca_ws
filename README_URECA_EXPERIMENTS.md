# URECA Experiment Suite

Branch purpose: `ureca/experiment-suite` adds a reproducible TrajOpt experiment harness on top of the existing obstacle examples without changing the original runnable examples in place.

## Build

Use the repository's normal CMake or colcon workflow. One straightforward CMake flow is:

```bash
cmake -S . -B build
cmake --build build --target tesseract_examples_ureca_run_experiment_suite tesseract_examples_ureca_run_blackbox_demo
```

If you are building inside a ROS or colcon workspace, build the `tesseract_examples` package and then invoke the new executables from the resulting install or build space.

## Commands

General suite runner:

```bash
./build/tesseract_examples_ureca_run_experiment_suite \
  --config config/ureca_experiments.yaml \
  --experiment E1_2d_k_sweep \
  --output results
```

Black-box demo:

```bash
./build/tesseract_examples_ureca_run_blackbox_demo \
  --config config/ureca_experiments.yaml \
  --output results/E8_blackbox_wrapper_demo
```

Per-experiment runner examples:

```bash
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E0_reproduce_current --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E1_2d_k_sweep --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E2_3d_k_sweep --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E3_no_subgoal_vs_subgoal --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E4_seed_penetration_depth --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E5_symmetry_breaking --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E6_obstacle_difficulty_sweep --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E7_subgoal_location_sweep --output results
./build/tesseract_examples_ureca_run_experiment_suite --config config/ureca_experiments.yaml --experiment E8_blackbox_wrapper_demo --output results
```

Generate plots:

```bash
python3 scripts/ureca_plot_results.py --summary results/E1_2d_k_sweep/summary.csv
python3 scripts/ureca_plot_results.py --summary results/E2_3d_k_sweep/summary.csv
python3 scripts/ureca_plot_results.py --summary results/E3_no_subgoal_vs_subgoal/comparison_table.csv
python3 scripts/ureca_plot_results.py --summary results/E4_seed_penetration_depth/penetration_summary.csv
python3 scripts/ureca_plot_results.py --summary results/E5_symmetry_breaking/symmetry_summary.csv
python3 scripts/ureca_plot_results.py --summary results/E6_obstacle_difficulty_sweep/obstacle_difficulty_summary.csv
python3 scripts/ureca_plot_results.py --summary results/E7_subgoal_location_sweep/subgoal_location_summary.csv
python3 scripts/ureca_plot_results.py --summary results/E8_blackbox_wrapper_demo/blackbox_demo_summary.csv
```

Collect result tables:

```bash
python3 scripts/ureca_collect_results.py --root results --output results/all_experiments.csv
```

## Experiment IDs

- `E0_reproduce_current`: reproduces the current 2D obstacle run and the current 3D subgoal timing sweep structure.
- `E1_2d_k_sweep`: sweeps subgoal timing in the new 2D subgoal experiment.
- `E2_3d_k_sweep`: sweeps subgoal timing in the 3D obstacle-subgoal experiment.
- `E3_no_subgoal_vs_subgoal`: compares no-subgoal, best-K, early-K, and late-K cases.
- `E4_seed_penetration_depth`: varies effective seed penetration depth by changing safety margin.
- `E5_symmetry_breaking`: perturbs symmetric seeds to test optimizer sensitivity.
- `E6_obstacle_difficulty_sweep`: sweeps obstacle radius and safety margin.
- `E7_subgoal_location_sweep`: moves the geometric subgoal while keeping timing fixed.
- `E8_blackbox_wrapper_demo`: runs the wrapper interface on the 3D obstacle-subgoal case.

## Output Schema

Summary CSV columns:

`experiment_id,case_id,dim,N,K,seed_type,perturbation,obstacle_radius,safe_radius,obstacle_offset,subgoal_x,subgoal_y,subgoal_z,seed_min_clearance,opt_min_clearance,max_penetration,objective,path_length,smoothness,solve_time_ms,status,success,trajectory_file`

Trajectory CSV columns:

`step,q0,q1,q2,x,y,z,clearance`

Derived metrics:

- `path_length = sum ||q[i+1] - q[i]||`
- `smoothness = sum ||q[i+1] - 2*q[i] + q[i-1]||^2`
- `max_penetration = max(0, -min_clearance)`

## Important Notes

- This branch is intentionally isolated from the original working branches.
- The original example sources are preserved; new functionality is added with the `ureca_` prefix.
- The copied URDF and SRDF files are used as templates for runtime-generated scenes so the original files stay untouched.
- DOF dimension must match the active manipulator group exactly.
- 2D coefficient vectors are always length 2.
- 3D coefficient vectors are always length 3.
- The new black-box wrapper preserves `request.format_result_as_input = false`.
- The current branch history only exposes a 2D obstacle example without a fixed subgoal, so the new 2D subgoal experiments use a configurable default subgoal in `config/ureca_experiments.yaml`.
- If TrajOpt cannot run in your local environment, do not commit generated numeric results. Keep only code, config, scripts, and empty result templates.
