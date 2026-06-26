#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_symmetry_failure_3d_example.hpp>
#include <tesseract_examples/ureca_result_utils.hpp>

#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/stopwatch.h>

#include <tesseract_environment/environment.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/state_waypoint.h>

#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

using namespace tesseract_environment;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract_examples
{
namespace
{
constexpr double OBSTACLE_RADIUS = 0.20;
constexpr double ROBOT_RADIUS = 0.10;
const Eigen::Vector3d OBSTACLE_CENTER(0.5, 0.5, 0.0);

bool extractTrajectory(const CompositeInstruction& result, std::vector<Eigen::VectorXd>& trajectory)
{
  trajectory.clear();
  auto flattened = result.flatten();
  for (const auto& instr_ref : flattened)
  {
    const auto& instr = instr_ref.get();
    if (!instr.isMoveInstruction())
      continue;
    const auto& move = instr.as<MoveInstructionPoly>();
    const auto& wp = move.getWaypoint();
    if (wp.isStateWaypoint())
      trajectory.push_back(wp.as<StateWaypointPoly>().getPosition());
  }
  return !trajectory.empty();
}

Eigen::VectorXd interpolate(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double alpha)
{
  return a + alpha * (b - a);
}

std::vector<Eigen::VectorXd> makePerturbedSeed(const Eigen::VectorXd& start,
                                               const Eigen::VectorXd& subgoal,
                                               const Eigen::VectorXd& goal,
                                               int n,
                                               int k,
                                               double perturbation)
{
  std::vector<Eigen::VectorXd> seed;
  seed.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    Eigen::VectorXd q =
        (i <= k) ? interpolate(start, subgoal, static_cast<double>(i) / static_cast<double>(k))
                 : interpolate(subgoal, goal, static_cast<double>(i - k) / static_cast<double>(n - 1 - k));
    if (i > 0 && i < n - 1)
      q(2) += perturbation;
    seed.push_back(q);
  }
  return seed;
}

CompositeInstruction buildProgram(const std::vector<std::string>& joint_names,
                                  const std::vector<Eigen::VectorXd>& seed,
                                  int k)
{
  CompositeInstruction program("ureca_symmetry_failure_3d_program",
                               ManipulatorInfo("manipulator", "base_link", "robot_body"));
  for (int i = 0; i < static_cast<int>(seed.size()); ++i)
  {
    StateWaypoint wp{ joint_names, seed[static_cast<std::size_t>(i)] };
    const std::string profile = (i == 0 || i == k || i == static_cast<int>(seed.size()) - 1) ? "fixed_profile" : "seed_profile";
    MoveInstruction move(wp, MoveInstructionType::FREESPACE, profile);
    program.push_back(move);
  }
  return program;
}

std::shared_ptr<tesseract_common::ProfileDictionary> createProfiles()
{
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.02, 1000);
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
  composite_profile->collision_constraint_config.enabled = true;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(3) * 0.01;
  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "ureca_symmetry_failure_3d_program", composite_profile);

  auto fixed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_profile->cartesian_cost_config.enabled = false;
  fixed_profile->cartesian_constraint_config.enabled = false;
  fixed_profile->joint_cost_config.enabled = false;
  fixed_profile->joint_constraint_config.enabled = true;
  fixed_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(3);
  profiles->addProfile(TRAJOPT_NAMESPACE, "fixed_profile", fixed_profile);

  auto seed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  seed_profile->cartesian_cost_config.enabled = false;
  seed_profile->cartesian_constraint_config.enabled = false;
  seed_profile->joint_cost_config.enabled = false;
  seed_profile->joint_constraint_config.enabled = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "seed_profile", seed_profile);
  return profiles;
}

}  // namespace

UrecaSymmetryFailure3DExample::UrecaSymmetryFailure3DExample(std::shared_ptr<tesseract_environment::Environment> env,
                                                             std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                                             bool ifopt,
                                                             bool debug)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
{
}

bool UrecaSymmetryFailure3DExample::run()
{
  console_bridge::setLogLevel(debug_ ? console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG :
                                       console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  const std::vector<std::string> joint_names{ "joint_x", "joint_y", "joint_z" };
  Eigen::VectorXd start(3);
  start << 0.0, 0.0, 0.0;
  Eigen::VectorXd goal(3);
  goal << 2.0, 1.0, 0.0;
  Eigen::VectorXd subgoal(3);
  subgoal << 1.0, 1.0, 0.0;
  const int n = 25;
  const int k = 12;
  const std::vector<double> perturbations{ 0.00, 0.01, -0.01, 0.03, -0.03, 0.05, -0.05 };

  env_->setState(joint_names, start);
  if (!env_->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set BulletDiscreteBVHManager.");
    return false;
  }

  const std::filesystem::path result_dir("/home/wenda/tesseract_ws/result_ureca/symmetry_failure_3d");
  urecaEnsureResultLayout(result_dir);
  auto profiles = createProfiles();
  TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);

  std::vector<UrecaSummaryRow> rows;
  for (double perturbation : perturbations)
  {
    const std::vector<Eigen::VectorXd> seed = makePerturbedSeed(start, subgoal, goal, n, k, perturbation);
    CompositeInstruction program = buildProgram(joint_names, seed, k);

    PlannerRequest request;
    request.instructions = program;
    request.env = env_;
    request.profiles = profiles;
    request.verbose = false;
    request.format_result_as_input = false;

    tesseract_common::Stopwatch stopwatch;
    stopwatch.start();
    PlannerResponse response = planner.solve(request);
    stopwatch.stop();

    UrecaSummaryRow row;
    row.case_id = "perturb_" + std::to_string(perturbation);
    row.K = k;
    row.perturbation = perturbation;
    row.trajopt_success = response.successful;
    row.trajopt_message = response.message;
    row.solve_time_ms = stopwatch.elapsedSeconds() * 1000.0;

    if (response.successful)
    {
      std::vector<Eigen::VectorXd> opt_traj;
      if (extractTrajectory(response.results, opt_traj))
      {
        const double min_clearance =
            urecaComputeMinSphereObstacleClearanceProxy(opt_traj, OBSTACLE_CENTER, OBSTACLE_RADIUS, ROBOT_RADIUS, 0, 3);
        const auto per_step =
            urecaComputePerStepObstacleClearanceProxy(opt_traj, OBSTACLE_CENTER, OBSTACLE_RADIUS, ROBOT_RADIUS, 0, 3);
        row.collision_free = (min_clearance >= 0.0);
        row.post_path_length = urecaComputePathLength(opt_traj);
        row.post_smoothness_proxy = urecaComputeSmoothnessProxy(opt_traj);
        row.post_min_clearance_proxy = min_clearance;
        row.post_min_obstacle_clearance_proxy = min_clearance;
        row.post_min_inter_robot_clearance_proxy = std::numeric_limits<double>::quiet_NaN();
        row.trajectory_file = row.case_id + ".csv";
        urecaWriteSingleRobotTrajectoryCsv(result_dir / "trajectories" / row.trajectory_file, opt_traj, per_step, 3);
      }
    }

    rows.push_back(row);
  }

  urecaWriteSummaryCsv(result_dir / "summary.csv", rows);
  return true;
}

}  // namespace tesseract_examples
