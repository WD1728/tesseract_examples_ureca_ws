/**
 * @file 3d_example.cpp
 * @brief 3D single-robot TrajOpt subgoal-timing experiment with obstacle avoidance.
 *
 * This example is a 3D sanity-check version of the earlier 2D toy problem.
 * The robot has three prismatic joints: joint_x, joint_y, and joint_z.
 * The robot body is represented as a small sphere, and the static obstacle is
 * represented as a larger sphere placed on the straight-line path.
 *
 * The experiment constructs a fixed-length trajectory q_0 ... q_{N-1}.
 * The start q_0, subgoal q_k, and goal q_{N-1} are enforced using
 * joint equality constraints through "fixed_profile". All other intermediate
 * states are seed-only waypoints. The code sweeps k and solves one TrajOpt
 * problem for each candidate subgoal timestep.
 *
 * This version directly calls TrajOptMotionPlanner rather than TaskComposer,
 * so the program structure and number of timesteps are controlled explicitly.
 *
 * Outputs are written to:
 *   /home/wenda/tesseract_ws/result_3d/
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/3d_example.h>

#include <algorithm>
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
constexpr double OBSTACLE_Y = 0.5;
constexpr double OBSTACLE_Z = 0.0;
constexpr double OBSTACLE_RADIUS = 0.20;
constexpr double ROBOT_RADIUS = 0.10;
constexpr double SAFE_RADIUS = OBSTACLE_RADIUS + ROBOT_RADIUS;

double computePathLengthObjective(const std::vector<Eigen::VectorXd>& traj)
{
  double objective = 0.0;

  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
    objective += (traj[i + 1] - traj[i]).squaredNorm();

  return objective;
}

double pointSegmentDistance3D(const Eigen::Vector3d& p,
                              const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b)
{
  const Eigen::Vector3d ab = b - a;
  const double denom = ab.squaredNorm();

  if (denom < 1e-12)
    return (p - a).norm();

  double t = (p - a).dot(ab) / denom;
  t = std::max(0.0, std::min(1.0, t));

  const Eigen::Vector3d closest = a + t * ab;
  return (p - closest).norm();
}

double computeMinClearance3D(const std::vector<Eigen::VectorXd>& traj)
{
  const Eigen::Vector3d obstacle_center(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z);
  double min_dist = std::numeric_limits<double>::infinity();

  for (const auto& q_vec : traj)
  {
    if (q_vec.size() < 3)
      continue;

    const Eigen::Vector3d q(q_vec(0), q_vec(1), q_vec(2));
    min_dist = std::min(min_dist, (q - obstacle_center).norm());
  }

  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
  {
    if (traj[i].size() < 3 || traj[i + 1].size() < 3)
      continue;

    const Eigen::Vector3d a(traj[i](0), traj[i](1), traj[i](2));
    const Eigen::Vector3d b(traj[i + 1](0), traj[i + 1](1), traj[i + 1](2));
    min_dist = std::min(min_dist, pointSegmentDistance3D(obstacle_center, a, b));
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

ThreeDExample::ThreeDExample(std::shared_ptr<tesseract_environment::Environment> env,
                             std::shared_ptr<tesseract_visualization::Visualization> plotter,
                             bool ifopt,
                             bool debug)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
{
}

bool ThreeDExample::run()
{
  if (debug_)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  std::vector<std::string> joint_names{ "joint_x", "joint_y", "joint_z" };

  Eigen::VectorXd start(3);
  start << 0.0, 0.0, 0.0;

  Eigen::VectorXd goal(3);
  goal << 2.0, 1.0, 0.0;

  Eigen::VectorXd subgoal(3);
  subgoal << 1.0, 1.2, 0.0;

  env_->setState(joint_names, start);

  if (!env_->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set active discrete contact manager.");
    return false;
  }

  const int N = 20;

  const double vel_coeff = 0.01;

  const std::string result_dir = "/home/wenda/tesseract_ws/result_3d/";
  std::filesystem::create_directories(result_dir);

  std::ofstream summary_file(result_dir + "cost_vs_k.txt");
  if (!summary_file.is_open())
  {
    CONSOLE_BRIDGE_logError("Failed to open result_3d/cost_vs_k.txt");
    return false;
  }

  summary_file << "# k path_objective solver_success min_clearance collision_free\n";

  double best_objective = std::numeric_limits<double>::infinity();
  double best_clearance = -std::numeric_limits<double>::infinity();
  int best_k = -1;
  std::vector<Eigen::VectorXd> best_traj;

  for (int subgoal_step = 1; subgoal_step < N - 1; ++subgoal_step)
  {
    std::cout << "\n===== Testing subgoal_step = " << subgoal_step << " =====\n";

    env_->setState(joint_names, start);

    CompositeInstruction program("subgoal_program",
                                 ManipulatorInfo("manipulator", "base_link", "robot_body"));

    StateWaypoint wp_start{ joint_names, start };
    MoveInstruction start_instruction(wp_start, MoveInstructionType::FREESPACE, "fixed_profile");
    start_instruction.setDescription("start");
    program.push_back(start_instruction);

    for (int i = 1; i < N - 1; ++i)
    {
      if (i == subgoal_step)
      {
        StateWaypoint wp_subgoal{ joint_names, subgoal };
        MoveInstruction subgoal_instruction(wp_subgoal, MoveInstructionType::FREESPACE, "fixed_profile");
        subgoal_instruction.setDescription("subgoal_k_" + std::to_string(subgoal_step));
        program.push_back(subgoal_instruction);
      }
      else
      {
        const double a = static_cast<double>(i) / static_cast<double>(N - 1);
        Eigen::VectorXd q = start + a * (goal - start);

        const double pi = 3.14159265358979323846;
        // Add a small sideways bias so the initial trajectory does not pass through
        // the obstacle center in the xy projection.
        // The direction is perpendicular to the start-goal line.
        // start -> goal direction is (2, 1), so a perpendicular direction is (-1, 2).
        Eigen::Vector3d side_dir(-1.0, 2.0, 0.0);
        side_dir.normalize();

        // Keep the bias small enough so the seed is still in collision,
        const double side_bias = 0.08 * std::sin(pi * a);
        q(0) += side_bias * side_dir(0);
        q(1) += side_bias * side_dir(1);

        // Optional small z bias, also still much smaller than SAFE_RADIUS.
        q(2) += 0.05 * std::sin(pi * a);

        StateWaypoint wp_seed{ joint_names, q };
        MoveInstruction seed_instruction(wp_seed, MoveInstructionType::FREESPACE, "seed_profile");
        seed_instruction.setDescription("seed_" + std::to_string(i));
        program.push_back(seed_instruction);
      }
    }

    StateWaypoint wp_goal{ joint_names, goal };
    MoveInstruction goal_instruction(wp_goal, MoveInstructionType::FREESPACE, "fixed_profile");
    goal_instruction.setDescription("goal");
    program.push_back(goal_instruction);

    if (debug_)
      program.print("Program: ");

    auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

    auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

    // soft collision cost 
    composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.0025, 1000);
    composite_profile->collision_cost_config.enabled = true;
    composite_profile->collision_cost_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

    // hard collision constraints 
    composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
    composite_profile->collision_constraint_config.enabled = false;
    composite_profile->collision_constraint_config.collision_check_config.type =
        tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

    composite_profile->smooth_velocities = true;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(3) * vel_coeff;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;

    profiles->addProfile(TRAJOPT_NAMESPACE, "subgoal_program", composite_profile);

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

    TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);

    PlannerRequest request;
    request.instructions = program;
    request.env = env_;
    request.profiles = profiles;
    request.verbose = false;
    //must set as false, otherwie cannot get final trajectory. 
    request.format_result_as_input = false;

    tesseract_common::Stopwatch stopwatch;
    stopwatch.start();

    PlannerResponse response = planner.solve(request);

    stopwatch.stop();

    if (!response.successful)
    {
      CONSOLE_BRIDGE_logWarn("TrajOpt failed at k = %d: %s", subgoal_step, response.message.c_str());

      summary_file << subgoal_step << " " << -1.0 << " " << 0 << " " << -1.0 << " " << 0 << '\n';
      continue;
    }

    CONSOLE_BRIDGE_logInform("Planning for k = %d took %f seconds.", subgoal_step, stopwatch.elapsedSeconds());

    const CompositeInstruction& result = response.results;

    std::vector<Eigen::VectorXd> traj;
    if (!extractTrajectory(result, traj))
    {
      CONSOLE_BRIDGE_logWarn("Failed to extract trajectory at k = %d", subgoal_step);

      summary_file << subgoal_step << " " << -1.0 << " " << 0 << " " << -1.0 << " " << 0 << '\n';
      continue;
    }

    if (static_cast<int>(traj.size()) != N)
    {
      CONSOLE_BRIDGE_logWarn("Unexpected trajectory size at k = %d: got %zu, expected %d",
                             subgoal_step,
                             traj.size(),
                             N);
    }

    const double path_objective = vel_coeff * computePathLengthObjective(traj);
    const double min_clearance = computeMinClearance3D(traj);
    const bool collision_free = (min_clearance >= 0.0);

    std::cout << "k = " << subgoal_step
              << ", path_objective = " << path_objective
              << ", min_clearance = " << min_clearance
              << ", collision_free = " << collision_free
              << std::endl;

    summary_file << subgoal_step << " "
                 << path_objective << " "
                 << 1 << " "
                 << min_clearance << " "
                 << static_cast<int>(collision_free) << '\n';

    saveTrajectoryToFile(traj, result_dir + "trajectory_k_" + std::to_string(subgoal_step) + ".txt");

    if (collision_free && path_objective < best_objective)
    {
      best_objective = path_objective;
      best_clearance = min_clearance;
      best_k = subgoal_step;
      best_traj = traj;
    }
  }

  summary_file.close();

  if (best_k >= 0)
  {
    std::cout << "\n===== Best Result =====\n";
    std::cout << "best_k = " << best_k
              << ", best_objective = " << best_objective
              << ", best_clearance = " << best_clearance
              << std::endl;

    saveTrajectoryToFile(best_traj, result_dir + "trajectory_best.txt");

    std::ofstream best_file(result_dir + "best_result.txt");
    if (best_file.is_open())
    {
      best_file << "best_k " << best_k << '\n';
      best_file << "best_objective " << best_objective << '\n';
      best_file << "best_clearance " << best_clearance << '\n';
      best_file.close();
    }

    std::cout << "Saved results to " << result_dir << std::endl;
    return true;
  }

  CONSOLE_BRIDGE_logError("No collision-free trajectory found in subgoal step sweep.");
  return false;
}

}  // namespace tesseract_examples
