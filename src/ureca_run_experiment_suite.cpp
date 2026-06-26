#include <tesseract_examples/ureca_experiment_config.hpp>
#include <tesseract_examples/ureca_experiment_metrics.hpp>
#include <tesseract_examples/ureca_experiment_utils.hpp>
#include <tesseract_examples/ureca_trajopt_blackbox.hpp>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>

namespace tesseract_examples
{
namespace
{
struct CliArguments
{
  std::filesystem::path config_path;
  std::string experiment_id;
  std::filesystem::path output_dir;
};

CliArguments parseArgs(int argc, char** argv)
{
  CliArguments args;

  for (int i = 1; i < argc; ++i)
  {
    const std::string token = argv[i];
    if (token == "--config" && i + 1 < argc)
      args.config_path = argv[++i];
    else if (token == "--experiment" && i + 1 < argc)
      args.experiment_id = argv[++i];
    else if (token == "--output" && i + 1 < argc)
      args.output_dir = argv[++i];
    else
      throw std::runtime_error("Unknown or incomplete argument: " + token);
  }

  if (args.config_path.empty() || args.experiment_id.empty() || args.output_dir.empty())
    throw std::runtime_error("Usage: ureca_run_experiment_suite --config <yaml> --experiment <id> --output <dir>");

  return args;
}

TrajOptResult runCase(const UrecaSceneConfig& scene,
                      const std::filesystem::path& runtime_dir,
                      const std::vector<Eigen::VectorXd>& seed,
                      const std::optional<Eigen::VectorXd>& subgoal,
                      int k)
{
  std::string env_status;
  auto env = createEnvironmentForScene(scene, runtime_dir, env_status);
  TrajOptResult result;
  if (env == nullptr)
  {
    result.success = false;
    result.status_message = env_status;
    result.seed_trajectory = seed;
    result.seed_min_clearance = computeUrecaMinClearance(scene, seed);
    return result;
  }

  UrecaTrajOptRequest request;
  request.environment = env;
  request.scene = scene;
  request.q_start = scene.start;
  request.q_goal = scene.goal;
  request.q_subgoal = subgoal;
  request.subgoal_step = k;
  request.n_steps = scene.n_steps;
  request.seed = seed;
  return runTrajOptBlackBox(request);
}

void saveResultArtifacts(const UrecaSceneConfig& scene,
                         const std::filesystem::path& output_dir,
                         const std::string& stem,
                         const TrajOptResult& result,
                         UrecaSummaryRow& row)
{
  if (!result.seed_trajectory.empty())
    writeUrecaTrajectoryCsv(output_dir / (stem + "_seed.csv"), scene, result.seed_trajectory);

  const std::filesystem::path trajectory_path = output_dir / (stem + "_optimized.csv");
  if (result.success && !result.optimized_trajectory.empty())
  {
    writeUrecaTrajectoryCsv(trajectory_path, scene, result.optimized_trajectory);
    row.trajectory_file = trajectory_path.filename().string();
  }
  else
  {
    row.trajectory_file.clear();
  }
}

void runE1orE2(const UrecaExperimentContext& context,
               const std::string& experiment_id,
               UrecaSceneConfig scene,
               const std::optional<Eigen::VectorXd>& subgoal)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, experiment_id);
  std::vector<UrecaSummaryRow> rows;

  for (int k = 1; k < scene.n_steps - 1; ++k)
  {
    const std::vector<Eigen::VectorXd> seed = makeSubgoalSeed(scene.start, *subgoal, scene.goal, scene.n_steps, k);
    TrajOptResult result = runCase(scene, context.runtime_dir / experiment_id / ("k_" + std::to_string(k)), seed, subgoal, k);
    UrecaSummaryRow row =
        makeSummaryRow(experiment_id, "K_" + std::to_string(k), scene, k, "piecewise_subgoal", 0.0, 0.0, subgoal, result, "");
    saveResultArtifacts(scene, dir, "K_" + std::to_string(k), result, row);
    rows.push_back(row);
  }

  writeUrecaSummaryCsv(dir / "summary.csv", rows);
}

void runE0(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E0_reproduce_current");
  std::vector<UrecaSummaryRow> rows;

  UrecaSceneConfig two_d = makeBase2DScene(context.suite_config, context.workspace_root);
  {
    const std::vector<Eigen::VectorXd> seed = makeLinearSeed(two_d.start, two_d.goal, two_d.n_steps);
    TrajOptResult result = runCase(two_d, context.runtime_dir / "E0/2d", seed, std::nullopt, -1);
    UrecaSummaryRow row =
        makeSummaryRow("E0_reproduce_current", "2d_current", two_d, -1, "linear", 0.0, 0.0, std::nullopt, result, "");
    saveResultArtifacts(two_d, dir, "2d_current", result, row);
    rows.push_back(row);
  }

  UrecaSceneConfig three_d = makeBase3DScene(context.suite_config, context.workspace_root);
  const int start_k = 1;
  const int end_k = three_d.n_steps - 2;
  for (int k = start_k; k <= end_k; ++k)
  {
    const std::vector<Eigen::VectorXd> seed =
        makeSubgoalSeed(three_d.start, *three_d.subgoal, three_d.goal, three_d.n_steps, k);
    TrajOptResult result = runCase(three_d, context.runtime_dir / "E0/3d" / ("k_" + std::to_string(k)), seed, three_d.subgoal, k);
    UrecaSummaryRow row =
        makeSummaryRow("E0_reproduce_current", "3d_K_" + std::to_string(k), three_d, k, "piecewise_subgoal", 0.0, 0.0, three_d.subgoal, result, "");
    saveResultArtifacts(three_d, dir, "3d_K_" + std::to_string(k), result, row);
    rows.push_back(row);
  }

  writeUrecaSummaryCsv(dir / "summary.csv", rows);
}

void runE3(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E3_no_subgoal_vs_subgoal");
  std::vector<UrecaSummaryRow> rows;

  auto runDimension = [&](UrecaSceneConfig scene, const std::filesystem::path& best_summary_path, int fallback_best_k) {
    const int best_k =
        std::max(1, findBestSuccessfulK(best_summary_path) > 0 ? findBestSuccessfulK(best_summary_path) : fallback_best_k);
    const int early_k = clampSubgoalStep(context.suite_config.default_early_k, scene.n_steps);
    const int late_k = clampSubgoalStep(scene.n_steps - 1 - context.suite_config.default_late_k_offset, scene.n_steps);

    struct CaseConfig
    {
      std::string label;
      std::optional<Eigen::VectorXd> subgoal;
      int k;
      std::string seed_type;
    };

    const std::vector<CaseConfig> cases = {
      { "A_no_subgoal", std::nullopt, -1, "linear" },
      { "B_best_k", scene.subgoal, best_k, "piecewise_subgoal" },
      { "C_early_k", scene.subgoal, early_k, "piecewise_subgoal" },
      { "D_late_k", scene.subgoal, late_k, "piecewise_subgoal" },
    };

    for (const auto& case_config : cases)
    {
      const std::vector<Eigen::VectorXd> seed = case_config.subgoal.has_value()
                                                    ? makeSubgoalSeed(scene.start, *case_config.subgoal, scene.goal, scene.n_steps, case_config.k)
                                                    : makeLinearSeed(scene.start, scene.goal, scene.n_steps);
      TrajOptResult result = runCase(scene,
                                     context.runtime_dir / "E3" / scene.scene_id / case_config.label,
                                     seed,
                                     case_config.subgoal,
                                     case_config.k);
      UrecaSummaryRow row =
          makeSummaryRow("E3_no_subgoal_vs_subgoal",
                         scene.scene_id + "_" + case_config.label,
                         scene,
                         case_config.k,
                         case_config.seed_type,
                         0.0,
                         0.0,
                         case_config.subgoal,
                         result,
                         "");
      saveResultArtifacts(scene, dir, scene.scene_id + "_" + case_config.label, result, row);
      rows.push_back(row);
    }
  };

  runDimension(makeBase2DScene(context.suite_config, context.workspace_root),
               context.output_dir / "E1_2d_k_sweep/summary.csv",
               context.suite_config.default_selected_k_2d);
  runDimension(makeBase3DScene(context.suite_config, context.workspace_root),
               context.output_dir / "E2_3d_k_sweep/summary.csv",
               context.suite_config.default_selected_k_3d);

  writeUrecaSummaryCsv(dir / "comparison_table.csv", rows);
}

void runE4(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E4_seed_penetration_depth");
  std::vector<UrecaSummaryRow> rows;

  auto runDimension = [&](UrecaSceneConfig scene, int selected_k) {
    const std::vector<Eigen::VectorXd> base_seed =
        makeSubgoalSeed(scene.start, *scene.subgoal, scene.goal, scene.n_steps, selected_k);
    const double base_clearance = computeUrecaMinClearance(scene, base_seed);

    for (double target : context.suite_config.penetration_targets)
    {
      UrecaSceneConfig case_scene = scene;
      case_scene.safe_margin = std::max(0.0, base_clearance - target);
      case_scene.collision_cost_margin = case_scene.safe_margin;
      case_scene.collision_constraint_margin = case_scene.safe_margin;
      TrajOptResult result =
          runCase(case_scene, context.runtime_dir / "E4" / scene.scene_id / formatOptionalCoordinate(scene.subgoal, 0), base_seed, scene.subgoal, selected_k);
      UrecaSummaryRow row =
          makeSummaryRow("E4_seed_penetration_depth",
                         scene.scene_id + "_target_" + std::to_string(target),
                         case_scene,
                         selected_k,
                         "piecewise_subgoal",
                         0.0,
                         0.0,
                         scene.subgoal,
                         result,
                         "");
      saveResultArtifacts(case_scene, dir, row.case_id, result, row);
      rows.push_back(row);
    }
  };

  runDimension(makeBase2DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_2d);
  runDimension(makeBase3DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_3d);

  writeUrecaSummaryCsv(dir / "penetration_summary.csv", rows);
}

void runE5(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E5_symmetry_breaking");
  std::vector<UrecaSummaryRow> rows;

  auto runDimension = [&](UrecaSceneConfig scene, int selected_k, int perturb_axis) {
    UrecaSceneConfig symmetric_scene = scene;
    symmetric_scene.obstacle_center = 0.5 * (Eigen::Vector3d(scene.start(0), scene.start(1), scene.dim > 2 ? scene.start(2) : 0.0) +
                                             Eigen::Vector3d(scene.goal(0), scene.goal(1), scene.dim > 2 ? scene.goal(2) : 0.0));

    const std::vector<Eigen::VectorXd> base_seed =
        makeSubgoalSeed(scene.start, *scene.subgoal, scene.goal, scene.n_steps, selected_k);

    for (double perturbation : context.suite_config.symmetry_perturbations)
    {
      const std::vector<Eigen::VectorXd> seed = perturbSeed(base_seed, perturb_axis, perturbation);
      TrajOptResult result = runCase(symmetric_scene,
                                     context.runtime_dir / "E5" / scene.scene_id / ("p_" + std::to_string(perturbation)),
                                     seed,
                                     scene.subgoal,
                                     selected_k);
      UrecaSummaryRow row =
          makeSummaryRow("E5_symmetry_breaking",
                         scene.scene_id + "_perturb_" + std::to_string(perturbation),
                         symmetric_scene,
                         selected_k,
                         "perturbed_subgoal",
                         perturbation,
                         0.0,
                         scene.subgoal,
                         result,
                         "");
      saveResultArtifacts(symmetric_scene, dir, row.case_id, result, row);
      rows.push_back(row);
    }
  };

  runDimension(makeBase2DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_2d, 1);
  runDimension(makeBase3DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_3d, 2);

  writeUrecaSummaryCsv(dir / "symmetry_summary.csv", rows);
}

void runE6(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E6_obstacle_difficulty_sweep");
  std::vector<UrecaSummaryRow> rows;

  auto runDimension = [&](UrecaSceneConfig scene, int selected_k) {
    const std::vector<Eigen::VectorXd> seed =
        makeSubgoalSeed(scene.start, *scene.subgoal, scene.goal, scene.n_steps, selected_k);

    for (double radius : context.suite_config.obstacle_radius_values)
    {
      for (double safe_margin : context.suite_config.safe_margin_values)
      {
        UrecaSceneConfig case_scene = scene;
        case_scene.obstacle_radius = radius;
        case_scene.safe_margin = safe_margin;
        case_scene.collision_cost_margin = safe_margin;
        case_scene.collision_constraint_margin = safe_margin;

        TrajOptResult result = runCase(case_scene,
                                       context.runtime_dir / "E6" / scene.scene_id / ("r_" + std::to_string(radius) + "_m_" + std::to_string(safe_margin)),
                                       seed,
                                       scene.subgoal,
                                       selected_k);
        UrecaSummaryRow row =
            makeSummaryRow("E6_obstacle_difficulty_sweep",
                           scene.scene_id + "_r_" + std::to_string(radius) + "_m_" + std::to_string(safe_margin),
                           case_scene,
                           selected_k,
                           "piecewise_subgoal",
                           0.0,
                           0.0,
                           scene.subgoal,
                           result,
                           "");
        saveResultArtifacts(case_scene, dir, row.case_id, result, row);
        rows.push_back(row);
      }
    }
  };

  runDimension(makeBase2DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_2d);
  runDimension(makeBase3DScene(context.suite_config, context.workspace_root), context.suite_config.default_selected_k_3d);

  writeUrecaSummaryCsv(dir / "obstacle_difficulty_summary.csv", rows);
}

void runE7(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E7_subgoal_location_sweep");
  std::vector<UrecaSummaryRow> rows;

  UrecaSceneConfig two_d = makeBase2DScene(context.suite_config, context.workspace_root);
  for (double subgoal_y : context.suite_config.two_d_subgoal_y_values)
  {
    UrecaSceneConfig case_scene = two_d;
    Eigen::VectorXd subgoal = *two_d.subgoal;
    subgoal(1) = subgoal_y;
    case_scene.subgoal = subgoal;
    const std::vector<Eigen::VectorXd> seed =
        makeSubgoalSeed(case_scene.start, subgoal, case_scene.goal, case_scene.n_steps, context.suite_config.default_selected_k_2d);
    TrajOptResult result =
        runCase(case_scene, context.runtime_dir / "E7/2d" / ("y_" + std::to_string(subgoal_y)), seed, case_scene.subgoal, context.suite_config.default_selected_k_2d);
    UrecaSummaryRow row =
        makeSummaryRow("E7_subgoal_location_sweep",
                       "2d_subgoal_y_" + std::to_string(subgoal_y),
                       case_scene,
                       context.suite_config.default_selected_k_2d,
                       "piecewise_subgoal",
                       0.0,
                       0.0,
                       case_scene.subgoal,
                       result,
                       "");
    saveResultArtifacts(case_scene, dir, row.case_id, result, row);
    rows.push_back(row);
  }

  UrecaSceneConfig three_d = makeBase3DScene(context.suite_config, context.workspace_root);
  for (double subgoal_z : context.suite_config.three_d_subgoal_z_values)
  {
    UrecaSceneConfig case_scene = three_d;
    Eigen::VectorXd subgoal = *three_d.subgoal;
    subgoal(2) = subgoal_z;
    case_scene.subgoal = subgoal;
    const std::vector<Eigen::VectorXd> seed =
        makeSubgoalSeed(case_scene.start, subgoal, case_scene.goal, case_scene.n_steps, context.suite_config.default_selected_k_3d);
    TrajOptResult result =
        runCase(case_scene, context.runtime_dir / "E7/3d" / ("z_" + std::to_string(subgoal_z)), seed, case_scene.subgoal, context.suite_config.default_selected_k_3d);
    UrecaSummaryRow row =
        makeSummaryRow("E7_subgoal_location_sweep",
                       "3d_subgoal_z_" + std::to_string(subgoal_z),
                       case_scene,
                       context.suite_config.default_selected_k_3d,
                       "piecewise_subgoal",
                       0.0,
                       0.0,
                       case_scene.subgoal,
                       result,
                       "");
    saveResultArtifacts(case_scene, dir, row.case_id, result, row);
    rows.push_back(row);
  }

  writeUrecaSummaryCsv(dir / "subgoal_location_summary.csv", rows);
}

void runE8(const UrecaExperimentContext& context)
{
  const std::filesystem::path dir = ensureExperimentSubdir(context, "E8_blackbox_wrapper_demo");
  UrecaSceneConfig scene = makeBase3DScene(context.suite_config, context.workspace_root);
  const int k = clampSubgoalStep(context.suite_config.blackbox_demo_k, scene.n_steps);
  const std::vector<Eigen::VectorXd> seed = makeSubgoalSeed(scene.start, *scene.subgoal, scene.goal, scene.n_steps, k);
  TrajOptResult result = runCase(scene, context.runtime_dir / "E8/3d_demo", seed, scene.subgoal, k);

  UrecaSummaryRow row =
      makeSummaryRow("E8_blackbox_wrapper_demo", "blackbox_demo", scene, k, "piecewise_subgoal", 0.0, 0.0, scene.subgoal, result, "");
  saveResultArtifacts(scene, dir, "blackbox_demo", result, row);
  writeUrecaSummaryCsv(dir / "blackbox_demo_summary.csv", { row });
}

int runSuiteMain(int argc, char** argv)
{
  const CliArguments args = parseArgs(argc, argv);
  const std::vector<std::string> supported_ids = supportedUrecaExperimentIds();
  const std::set<std::string> supported(supported_ids.begin(), supported_ids.end());
  if (supported.find(args.experiment_id) == supported.end())
    throw std::runtime_error("Unsupported experiment id: " + args.experiment_id);

  UrecaExperimentContext context;
  const std::filesystem::path config_full_path = std::filesystem::absolute(args.config_path);
  context.workspace_root = std::filesystem::weakly_canonical(config_full_path.parent_path().parent_path());
  context.output_dir = std::filesystem::absolute(args.output_dir);
  context.runtime_dir = context.output_dir / "_runtime";
  context.suite_config = loadUrecaSuiteConfig(config_full_path);
  std::filesystem::create_directories(context.output_dir);

  if (args.experiment_id == "E0_reproduce_current")
    runE0(context);
  else if (args.experiment_id == "E1_2d_k_sweep")
  {
    const UrecaSceneConfig scene = makeBase2DScene(context.suite_config, context.workspace_root);
    runE1orE2(context, args.experiment_id, scene, scene.subgoal);
  }
  else if (args.experiment_id == "E2_3d_k_sweep")
  {
    const UrecaSceneConfig scene = makeBase3DScene(context.suite_config, context.workspace_root);
    runE1orE2(context, args.experiment_id, scene, scene.subgoal);
  }
  else if (args.experiment_id == "E3_no_subgoal_vs_subgoal")
    runE3(context);
  else if (args.experiment_id == "E4_seed_penetration_depth")
    runE4(context);
  else if (args.experiment_id == "E5_symmetry_breaking")
    runE5(context);
  else if (args.experiment_id == "E6_obstacle_difficulty_sweep")
    runE6(context);
  else if (args.experiment_id == "E7_subgoal_location_sweep")
    runE7(context);
  else if (args.experiment_id == "E8_blackbox_wrapper_demo")
    runE8(context);

  std::cout << "Finished " << args.experiment_id << " in " << context.output_dir << std::endl;
  return 0;
}

}  // namespace
}  // namespace tesseract_examples

int main(int argc, char** argv)
{
  try
  {
    return tesseract_examples::runSuiteMain(argc, argv);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("%s", e.what());
    return 1;
  }
}
