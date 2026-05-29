cat > /home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/src/2d_obstacle_example.cpp <<'EOF'
/**
 * @file 2d_obstacle_example.cpp
 * @brief Minimal 2D single-robot TrajOpt obstacle-avoidance sanity check.
 *
 * This version is intentionally kept as close as possible to the working 3D example:
 *   - 2 active joints only: joint_x, joint_y
 *   - start and goal are fixed by fixed_profile
 *   - intermediate waypoints are seed-only and free to move
 *   - initial seed slightly penetrates the obstacle
 *   - collision is handled by TrajOpt collision cost, not by manually calling contact manager APIs
 *
 * Outputs are written to:
 *   /home/wenda/tesseract_ws/result_2d_obstacle/
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/2d_obstacle_example.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/stopwatch.h>

#include <tesseract_environment/environment.h>

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>

using namespace tesseract_environment;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract_examples
{
namespace
{
constexpr double OBSTACLE_X = 1.0;
constexpr double OBSTACLE_Y = 0.78;
constexpr double OBSTACLE_RADIUS = 0.20;
constexpr double ROBOT_RADIUS = 0.10;
constexpr double SAFE_RADIUS = OBSTACLE_RADIUS + ROBOT_RADIUS;

double pointSegmentDistance2D(const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b)
{
  const Eigen::Vector2d ab = b - a;
  const double denom = ab.squaredNorm();

  if (denom < 1e-12)
    return (p - a).norm();

  double t = (p - a).dot(ab) / denom;
  t = std::max(0.0, std::min(1.0, t));

  const Eigen::Vector2d closest = a + t * ab;
  return (p - closest).norm();
}

double computePathLengthObjective(const std::vector<Eigen::VectorXd>& traj)
{
  double objective = 0.0;

  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
    objective += (traj[i + 1] - traj[i]).squaredNorm();

  return objective;
}

double computeMinClearance2D(const std::vector<Eigen::VectorXd>& traj)
{
  const Eigen::Vector2d obstacle_center(OBSTACLE_X, OBSTACLE_Y);
  double min_dist = std::numeric_limits<double>::infinity();

  for (const auto& q_vec : traj)
  {
    if (q_vec.size() < 2)
      continue;

    const Eigen::Vector2d q(q_vec(0), q_vec(1));
    min_dist = std::min(min_dist, (q - obstacle_center).norm());
  }

  // Also check segment-level clearance, not only waypoint-level clearance.
  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
  {
    if (traj[i].size() < 2 || traj[i + 1].size() < 2)
      continue;

    const Eigen::Vector2d a(traj[i](0), traj[i](1));
    const Eigen::Vector2d b(traj[i + 1](0), traj[i + 1](1));
    min_dist = std::min(min_dist, pointSegmentDistance2D(obstacle_center, a, b));
  }

  return min_dist - SAFE_RADIUS;
}

void saveTrajectoryToFile(const std::vector<Eigen::VectorXd>& traj, const std::string& filename)
{
  std::ofstream file(filename);
  if (!file.is_open())
  {
    std::cerr << "Failed to open trajectory output file: " << filename << std::endl;
    return;
  }

  for (const auto& q : traj)
    file << q.transpose() << '\n';
}

bool extractTrajectory(const CompositeInstruction& result, std::vector<Eigen::VectorXd>& traj)
{
  traj.clear();

  auto flattened = result.flatten();

  for (const auto& instr_ref : flattened)
  {
    const auto& instr = instr_ref.get();

    if (!instr.isMoveInstruction())
      continue;

    const auto& move_poly = instr.as<MoveInstructionPoly>();
    const auto& wp = move_poly.getWaypoint();

    if (wp.isStateWaypoint())
    {
      Eigen::VectorXd q = wp.as<StateWaypointPoly>().getPosition();
      traj.push_back(q);
    }
  }

  return !traj.empty();
}
}  // namespace

TwoDObstacleExample::TwoDObstacleExample(std::shared_ptr<tesseract_environment::Environment> env,
                                         std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                         bool ifopt,
                                         bool debug)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
{
}

bool TwoDObstacleExample::run()
{
  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  std::vector<std::string> joint_names{ "joint_x", "joint_y" };

  Eigen::VectorXd start(2);
  start << 0.0, 0.0;

  Eigen::VectorXd goal(2);
  goal << 2.0, 1.0;

  env_->setState(joint_names, start);

  if (!env_->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set active discrete contact manager: BulletDiscreteBVHManager.");
    return false;
  }

  const int N = 25;
  const double vel_coeff = 0.01;

  const std::string result_dir = "/home/wenda/tesseract_ws/result_2d_obstacle/";
  std::filesystem::create_directories(result_dir);

  CompositeInstruction program("two_d_obstacle_program",
                               ManipulatorInfo("manipulator", "base_link", "robot_body"));

  StateWaypoint wp_start{ joint_names, start };
  MoveInstruction start_instruction(wp_start, MoveInstructionType::FREESPACE, "fixed_profile");
  start_instruction.setDescription("start");
  program.push_back(start_instruction);

  // Seed:
  // Start-goal straight line is y = 0.5x.
  // At x = 1.0, the line passes through y = 0.5.
  // Obstacle center is at (1.0, 0.78).
  // The centerline distance is about 0.28 m.
  // Since SAFE_RADIUS = 0.30 m, the seed slightly penetrates the obstacle.
  for (int i = 1; i < N - 1; ++i)
  {
    const double a = static_cast<double>(i) / static_cast<double>(N - 1);
    Eigen::VectorXd q = start + a * (goal - start);

    StateWaypoint wp_seed{ joint_names, q };
    MoveInstruction seed_instruction(wp_seed, MoveInstructionType::FREESPACE, "seed_profile");
    seed_instruction.setDescription("seed_" + std::to_string(i));
    program.push_back(seed_instruction);
  }

  StateWaypoint wp_goal{ joint_names, goal };
  MoveInstruction goal_instruction(wp_goal, MoveInstructionType::FREESPACE, "fixed_profile");
  goal_instruction.setDescription("goal");
  program.push_back(goal_instruction);

  if (debug_)
    program.print("Program: ");

  std::vector<Eigen::VectorXd> seed_traj;
  if (!extractTrajectory(program, seed_traj))
  {
    CONSOLE_BRIDGE_logError("Failed to extract seed trajectory from input program.");
    return false;
  }

  const double seed_clearance = computeMinClearance2D(seed_traj);
  saveTrajectoryToFile(seed_traj, result_dir + "trajectory_seed.txt");

  std::cout << "seed_min_clearance = " << seed_clearance << " m" << std::endl;
  std::cout << "seed_collision_free = " << static_cast<int>(seed_clearance >= 0.0) << std::endl;

  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Keep this consistent with the working 3D example:
  // enable soft collision cost, disable hard collision constraint.
  composite_profile->collision_cost_config =
      trajopt_common::TrajOptCollisionConfig(0.0025, 1000);
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  composite_profile->collision_constraint_config =
      trajopt_common::TrajOptCollisionConfig(0.0, 20);
  composite_profile->collision_constraint_config.enabled = false;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(2) * vel_coeff;
  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;

  profiles->addProfile(TRAJOPT_NAMESPACE, "two_d_obstacle_program", composite_profile);

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

  TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);

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

  if (!response.successful)
  {
    CONSOLE_BRIDGE_logError("TrajOpt failed: %s", response.message.c_str());
    return false;
  }

  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());

  std::vector<Eigen::VectorXd> opt_traj;
  if (!extractTrajectory(response.results, opt_traj))
  {
    CONSOLE_BRIDGE_logError("Failed to extract optimized trajectory.");
    return false;
  }

  if (static_cast<int>(opt_traj.size()) != N)
  {
    CONSOLE_BRIDGE_logWarn("Unexpected trajectory size: got %zu, expected %d",
                           opt_traj.size(),
                           N);
  }

  const double opt_clearance = computeMinClearance2D(opt_traj);
  const bool opt_collision_free = (opt_clearance >= 0.0);
  const double opt_path_objective = vel_coeff * computePathLengthObjective(opt_traj);

  saveTrajectoryToFile(opt_traj, result_dir + "trajectory_optimized.txt");

  std::ofstream summary_file(result_dir + "summary.txt");
  if (summary_file.is_open())
  {
    summary_file << "seed_min_clearance " << seed_clearance << '\n';
    summary_file << "seed_collision_free " << static_cast<int>(seed_clearance >= 0.0) << '\n';
    summary_file << "opt_min_clearance " << opt_clearance << '\n';
    summary_file << "opt_collision_free " << static_cast<int>(opt_collision_free) << '\n';
    summary_file << "opt_path_objective " << opt_path_objective << '\n';
    summary_file.close();
  }

  std::cout << "opt_min_clearance = " << opt_clearance << " m" << std::endl;
  std::cout << "opt_collision_free = " << static_cast<int>(opt_collision_free) << std::endl;
  std::cout << "opt_path_objective = " << opt_path_objective << std::endl;
  std::cout << "Saved results to " << result_dir << std::endl;

  if (!opt_collision_free)
  {
    CONSOLE_BRIDGE_logWarn("Optimized trajectory is still in collision according to the external 2D clearance checker.");
  }

  return true;
}

}  // namespace tesseract_examples
EOF