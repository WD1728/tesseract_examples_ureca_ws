#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_subgoal_timing_2d_example.hpp>
#include <tesseract_examples/ureca_result_utils.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
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
constexpr double VELOCITY_COEFF = 0.01;
constexpr double ACCELERATION_COEFF = 0.01;
const Eigen::Vector2d OBSTACLE_CENTER(1.0, 0.6);

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

Eigen::VectorXd makeSeedPoint(int i,
                              int k,
                              int n,
                              const Eigen::VectorXd& start,
                              const Eigen::VectorXd& subgoal,
                              const Eigen::VectorXd& goal)
{
  if (i <= k)
    return interpolate(start, subgoal, static_cast<double>(i) / static_cast<double>(k));

  return interpolate(subgoal, goal, static_cast<double>(i - k) / static_cast<double>(n - 1 - k));
}

CompositeInstruction buildProgram(const std::vector<std::string>& joint_names,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& subgoal,
                                  const Eigen::VectorXd& goal,
                                  int n,
                                  int k)
{
  CompositeInstruction program("ureca_subgoal_timing_2d_program",
                               ManipulatorInfo("manipulator", "base_link", "robot_body"));

  for (int i = 0; i < n; ++i)
  {
    StateWaypoint wp{ joint_names, makeSeedPoint(i, k, n, start, subgoal, goal) };
    const std::string profile_name = (i == 0 || i == k || i == n - 1) ? "fixed_profile" : "seed_profile";
    MoveInstruction move(wp, MoveInstructionType::FREESPACE, profile_name);
    if (i == 0)
      move.setDescription("start");
    else if (i == k)
      move.setDescription("subgoal");
    else if (i == n - 1)
      move.setDescription("goal");
    else
      move.setDescription("seed_" + std::to_string(i));
    program.push_back(move);
  }

  return program;
}

std::shared_ptr<tesseract_common::ProfileDictionary> createProfiles()
{
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.0025, 1000);
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
  composite_profile->collision_constraint_config.enabled = false;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(2) * VELOCITY_COEFF;
  composite_profile->smooth_accelerations = true;
  composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(2) * ACCELERATION_COEFF;
  composite_profile->smooth_jerks = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "ureca_subgoal_timing_2d_program", composite_profile);

  auto fixed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_profile->cartesian_cost_config.enabled = false;
  fixed_profile->cartesian_constraint_config.enabled = false;
  fixed_profile->joint_cost_config.enabled = false;
  fixed_profile->joint_constraint_config.enabled = true;
  fixed_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(2);
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

UrecaSubgoalTiming2DExample::UrecaSubgoalTiming2DExample(std::shared_ptr<tesseract_environment::Environment> env,
                                                         std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                                         bool ifopt,
                                                         bool debug)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
{
}

bool UrecaSubgoalTiming2DExample::run()
{
  console_bridge::setLogLevel(debug_ ? console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG :
                                       console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  const std::vector<std::string> joint_names{ "joint_x", "joint_y" };
  Eigen::VectorXd start(2);
  start << 0.0, 0.0;
  Eigen::VectorXd goal(2);
  goal << 2.0, 1.0;
  Eigen::VectorXd subgoal(2);
  subgoal << 1.0, 1.05;

  env_->setState(joint_names, start);
  if (!env_->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set BulletDiscreteBVHManager.");
    return false;
  }

  const int n = 25;
  const std::filesystem::path result_dir("/home/wenda/tesseract_ws/result_ureca/subgoal_timing_2d");
  urecaEnsureResultLayout(result_dir);

  auto profiles = createProfiles();
  TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);
  std::vector<UrecaSummaryRow> rows;

  int best_k = -1;
  double best_clearance = -std::numeric_limits<double>::infinity();
  double best_path = std::numeric_limits<double>::infinity();

  for (int k = 1; k < n - 1; ++k)
  {
    CompositeInstruction program = buildProgram(joint_names, start, subgoal, goal, n, k);
    std::vector<Eigen::VectorXd> seed_traj;
    if (!extractTrajectory(program, seed_traj))
      continue;

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
    row.case_id = "K_" + std::to_string(k);
    row.K = k;
    row.trajopt_success = response.successful;
    row.trajopt_message = response.message;
    row.solve_time_ms = stopwatch.elapsedSeconds() * 1000.0;

    if (response.successful)
    {
      std::vector<Eigen::VectorXd> opt_traj;
      if (extractTrajectory(response.results, opt_traj))
      {
        const double min_obstacle_clearance = urecaComputeMinSphereObstacleClearanceProxy(
            opt_traj, Eigen::Vector3d(OBSTACLE_CENTER.x(), OBSTACLE_CENTER.y(), 0.0), OBSTACLE_RADIUS, ROBOT_RADIUS, 0, 2);
        const auto per_step = urecaComputePerStepObstacleClearanceProxy(
            opt_traj, Eigen::Vector3d(OBSTACLE_CENTER.x(), OBSTACLE_CENTER.y(), 0.0), OBSTACLE_RADIUS, ROBOT_RADIUS, 0, 2);

        row.collision_free = (min_obstacle_clearance >= 0.0);
        row.opt_path_objective = urecaComputePathObjective(opt_traj, VELOCITY_COEFF);
        row.post_path_length = urecaComputePathLength(opt_traj);
        row.post_smoothness_proxy = urecaComputeSmoothnessProxy(opt_traj);
        row.post_min_obstacle_clearance_proxy = min_obstacle_clearance;
        row.post_min_clearance_proxy = min_obstacle_clearance;
        row.post_min_inter_robot_clearance_proxy = std::numeric_limits<double>::quiet_NaN();
        row.trajectory_file = row.case_id + ".csv";

        urecaWriteSingleRobotTrajectoryCsv(result_dir / "trajectories" / row.trajectory_file, opt_traj, per_step, 2);

        if (row.collision_free &&
            (min_obstacle_clearance > best_clearance ||
             (std::abs(min_obstacle_clearance - best_clearance) < 1e-9 && row.post_path_length < best_path)))
        {
          best_k = k;
          best_clearance = min_obstacle_clearance;
          best_path = row.post_path_length;
        }
      }
    }

    rows.push_back(row);
  }

  urecaWriteSummaryCsv(result_dir / "summary.csv", rows);
  std::ofstream best_file(result_dir / "best_k.txt");
  if (best_file.is_open())
    best_file << best_k << "\n";

  return true;
}

}  // namespace tesseract_examples
