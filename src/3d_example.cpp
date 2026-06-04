/**
 * @file 3d_obstacle_example.cpp
 * @brief Experiment 1:
 *        Single 3D robot TrajOpt with one fixed subgoal at timestep K.
 *
 * Experiment:
 *   - 3 active joints only: joint_x, joint_y, joint_z
 *   - start and goal are fixed
 *   - one subgoal is fixed at timestep K
 *   - all other intermediate waypoints are seed-only and free to move
 *   - initial trajectory is piecewise linear: start -> subgoal -> goal
 *   - seed is allowed to collide with a static obstacle
 *   - TrajOpt collision cost tries to optimize the trajectory away from obstacle
 *   - K is enumerated from 1 to N-2
 *
 * Outputs:
 *   /home/wenda/tesseract_ws/result_3d_obstacle_subgoal/
 *     - subgoal_k_sweep.csv
 *     - seed_K_<K>.txt
 *     - opt_K_<K>.txt
 *     - summary_K_<K>.txt
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/3d_example.h>

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
constexpr double OBSTACLE_X = 0.5;
constexpr double OBSTACLE_Y = 0.6;
constexpr double OBSTACLE_Z = 0.0;

constexpr double OBSTACLE_RADIUS = 0.20;
constexpr double ROBOT_RADIUS = 0.10;
constexpr double SAFE_RADIUS = OBSTACLE_RADIUS + ROBOT_RADIUS;

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

double computePathLengthObjective(const std::vector<Eigen::VectorXd>& traj)
{
  double objective = 0.0;

  for (std::size_t i = 0; i + 1 < traj.size(); ++i)
    objective += (traj[i + 1] - traj[i]).squaredNorm();

  return objective;
}

double computeMinClearance3D(const std::vector<Eigen::VectorXd>& traj)
{
  const Eigen::Vector3d obstacle_center(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z);
  double min_dist = std::numeric_limits<double>::infinity();

  // Waypoint-level clearance
  for (const auto& q_vec : traj)
  {
    if (q_vec.size() < 3)
      continue;

    const Eigen::Vector3d q(q_vec(0), q_vec(1), q_vec(2));
    min_dist = std::min(min_dist, (q - obstacle_center).norm());
  }

  // Segment-level clearance
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

Eigen::VectorXd interpolate(const Eigen::VectorXd& a,
                            const Eigen::VectorXd& b,
                            double alpha)
{
  return a + alpha * (b - a);
}

Eigen::VectorXd makeSeedPointWithSubgoal(int i,
                                         int K,
                                         int N,
                                         const Eigen::VectorXd& start,
                                         const Eigen::VectorXd& subgoal,
                                         const Eigen::VectorXd& goal)
{
  if (i <= K)
  {
    const double alpha = static_cast<double>(i) / static_cast<double>(K);
    return interpolate(start, subgoal, alpha);
  }

  const double alpha = static_cast<double>(i - K) / static_cast<double>(N - 1 - K);
  return interpolate(subgoal, goal, alpha);
}

CompositeInstruction buildSingleRobotSubgoalProgram(const std::vector<std::string>& joint_names,
                                                    const Eigen::VectorXd& start,
                                                    const Eigen::VectorXd& subgoal,
                                                    const Eigen::VectorXd& goal,
                                                    int N,
                                                    int K)
{
  CompositeInstruction program("single_robot_subgoal_program",
                               ManipulatorInfo("manipulator", "base_link", "robot_body"));

  for (int i = 0; i < N; ++i)
  {
    Eigen::VectorXd q = makeSeedPointWithSubgoal(i, K, N, start, subgoal, goal);

    StateWaypoint wp{ joint_names, q };

    std::string profile_name;
    if (i == 0 || i == K || i == N - 1)
      profile_name = "fixed_profile";
    else
      profile_name = "seed_profile";

    MoveInstruction instruction(wp, MoveInstructionType::FREESPACE, profile_name);

    if (i == 0)
      instruction.setDescription("start");
    else if (i == K)
      instruction.setDescription("subgoal_K_" + std::to_string(K));
    else if (i == N - 1)
      instruction.setDescription("goal");
    else
      instruction.setDescription("seed_" + std::to_string(i));

    program.push_back(instruction);
  }

  return program;
}

std::shared_ptr<tesseract_common::ProfileDictionary>
createProfiles(double vel_coeff)
{

  constexpr int DOF = 3;

  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

  // Collision cost: safety margin, weight
  // 0.02 means TrajOpt starts penalizing when collision distance is below 2 cm.
  composite_profile->collision_cost_config =
      trajopt_common::TrajOptCollisionConfig(0.02, 1000);
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  // Collision constraint: safety margin, weight
  composite_profile->collision_constraint_config =
      trajopt_common::TrajOptCollisionConfig(0.0, 20);
  composite_profile->collision_constraint_config.enabled = true;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(3) * vel_coeff;

  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;

  profiles->addProfile(TRAJOPT_NAMESPACE,
                       "single_robot_subgoal_program",
                       composite_profile);

  auto fixed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_profile->cartesian_cost_config.enabled = false;
  fixed_profile->cartesian_constraint_config.enabled = false;
  fixed_profile->joint_cost_config.enabled = false;

  // Fixed waypoint:
  // joint_constraint_config enabled means this waypoint is constrained to stay at the provided joint position.
  fixed_profile->joint_constraint_config.enabled = true;
  fixed_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(3);

  profiles->addProfile(TRAJOPT_NAMESPACE,
                       "fixed_profile",
                       fixed_profile);

  auto seed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  seed_profile->cartesian_cost_config.enabled = false;
  seed_profile->cartesian_constraint_config.enabled = false;
  seed_profile->joint_cost_config.enabled = false;

  // Free waypoint:
  // This waypoint is used as the initial seed only.
  seed_profile->joint_constraint_config.enabled = false;

  profiles->addProfile(TRAJOPT_NAMESPACE,
                       "seed_profile",
                       seed_profile);

  return profiles;
}

void saveSummaryForK(const std::string& filename,
                     int K,
                     double seed_clearance,
                     bool seed_collision_free,
                     double opt_clearance,
                     bool opt_collision_free,
                     double opt_path_objective,
                     bool successful,
                     const std::string& message)
{
  std::ofstream summary_file(filename);
  if (!summary_file.is_open())
  {
    std::cerr << "Failed to open summary file: " << filename << std::endl;
    return;
  }

  summary_file << "K " << K << '\n';
  summary_file << "successful " << static_cast<int>(successful) << '\n';
  summary_file << "message " << message << '\n';
  summary_file << "seed_min_clearance " << seed_clearance << '\n';
  summary_file << "seed_collision_free " << static_cast<int>(seed_collision_free) << '\n';
  summary_file << "opt_min_clearance " << opt_clearance << '\n';
  summary_file << "opt_collision_free " << static_cast<int>(opt_collision_free) << '\n';
  summary_file << "opt_path_objective " << opt_path_objective << '\n';
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

  // Do not put the fixed subgoal inside the obstacle.
  // Obstacle center is (1.0, 0.6), obstacle radius is 0.20,
  // robot radius is 0.10, so the robot center must stay at least 0.30 m away.
  // Let obstacle pass through the seed between the start and subgoal. 
  Eigen::VectorXd subgoal(3);
  subgoal << 1.0, 1.05, 0.0;

  env_->setState(joint_names, start);

  if (!env_->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set active discrete contact manager: BulletDiscreteBVHManager.");
    return false;
  }

  const int N = 25;
  const double vel_coeff = 0.01;

  const std::string result_dir = "/home/wenda/tesseract_ws/result_3d_obstacle_subgoal/";
  std::filesystem::create_directories(result_dir);

  auto profiles = createProfiles(vel_coeff);

  TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);

  std::ofstream csv(result_dir + "subgoal_k_sweep.csv");
  if (!csv.is_open())
  {
    CONSOLE_BRIDGE_logError("Failed to open CSV output file.");
    return false;
  }

  csv << "K,"
      << "successful,"
      << "seed_min_clearance,"
      << "seed_collision_free,"
      << "opt_min_clearance,"
      << "opt_collision_free,"
      << "opt_path_objective,"
      << "solve_time_sec,"
      << "message\n";

  bool at_least_one_success = false;

  for (int K = 1; K < N - 1; ++K)
  {
    std::cout << "\n========== Running K = " << K << " ==========\n";

    CompositeInstruction program =
        buildSingleRobotSubgoalProgram(joint_names, start, subgoal, goal, N, K);

    if (debug_)
      program.print("Program K=" + std::to_string(K) + ": ");

    std::vector<Eigen::VectorXd> seed_traj;
    if (!extractTrajectory(program, seed_traj))
    {
      CONSOLE_BRIDGE_logError("Failed to extract seed trajectory for K=%d.", K);
      csv << K << ",0,nan,0,nan,0,nan,nan,failed_to_extract_seed\n";
      continue;
    }

    const double seed_clearance = computeMinClearance3D(seed_traj);
    const bool seed_collision_free = (seed_clearance >= 0.0);

    saveTrajectoryToFile(seed_traj,
                         result_dir + "seed_K_" + std::to_string(K) + ".txt");

    PlannerRequest request;
    request.instructions = program;
    request.env = env_;
    request.profiles = profiles;
    request.verbose = false;

    // Important: Keep false. 
    request.format_result_as_input = false;

    tesseract_common::Stopwatch stopwatch;
    stopwatch.start();

    PlannerResponse response = planner.solve(request);

    stopwatch.stop();

    const double solve_time = stopwatch.elapsedSeconds();

    if (!response.successful)
    {
      CONSOLE_BRIDGE_logWarn("TrajOpt failed for K=%d: %s",
                             K,
                             response.message.c_str());

      csv << K << ","
          << 0 << ","
          << seed_clearance << ","
          << static_cast<int>(seed_collision_free) << ","
          << "nan,"
          << 0 << ","
          << "nan,"
          << solve_time << ","
          << response.message << "\n";

      saveSummaryForK(result_dir + "summary_K_" + std::to_string(K) + ".txt",
                      K,
                      seed_clearance,
                      seed_collision_free,
                      std::numeric_limits<double>::quiet_NaN(),
                      false,
                      std::numeric_limits<double>::quiet_NaN(),
                      false,
                      response.message);

      continue;
    }

    std::vector<Eigen::VectorXd> opt_traj;
    if (!extractTrajectory(response.results, opt_traj))
    {
      CONSOLE_BRIDGE_logWarn("Failed to extract optimized trajectory for K=%d.", K);

      csv << K << ","
          << 0 << ","
          << seed_clearance << ","
          << static_cast<int>(seed_collision_free) << ","
          << "nan,"
          << 0 << ","
          << "nan,"
          << solve_time << ","
          << "failed_to_extract_optimized_trajectory\n";

      continue;
    }

    if (static_cast<int>(opt_traj.size()) != N)
    {
      CONSOLE_BRIDGE_logWarn("Unexpected trajectory size for K=%d: got %zu, expected %d",
                             K,
                             opt_traj.size(),
                             N);
    }

    const double opt_clearance = computeMinClearance3D(opt_traj);
    const bool opt_collision_free = (opt_clearance >= 0.0);
    const double opt_path_objective = vel_coeff * computePathLengthObjective(opt_traj);

    saveTrajectoryToFile(opt_traj,
                         result_dir + "opt_K_" + std::to_string(K) + ".txt");

    saveSummaryForK(result_dir + "summary_K_" + std::to_string(K) + ".txt",
                    K,
                    seed_clearance,
                    seed_collision_free,
                    opt_clearance,
                    opt_collision_free,
                    opt_path_objective,
                    true,
                    response.message);

    csv << K << ","
        << 1 << ","
        << seed_clearance << ","
        << static_cast<int>(seed_collision_free) << ","
        << opt_clearance << ","
        << static_cast<int>(opt_collision_free) << ","
        << opt_path_objective << ","
        << solve_time << ","
        << response.message << "\n";

    std::cout << "K = " << K << std::endl;
    std::cout << "seed_min_clearance = " << seed_clearance << " m" << std::endl;
    std::cout << "seed_collision_free = " << static_cast<int>(seed_collision_free) << std::endl;
    std::cout << "opt_min_clearance = " << opt_clearance << " m" << std::endl;
    std::cout << "opt_collision_free = " << static_cast<int>(opt_collision_free) << std::endl;
    std::cout << "opt_path_objective = " << opt_path_objective << std::endl;
    std::cout << "solve_time_sec = " << solve_time << std::endl;

    if (opt_collision_free)
      at_least_one_success = true;
    else
      CONSOLE_BRIDGE_logWarn("Optimized trajectory for K=%d is still in collision according to external checker.", K);
  }

  csv.close();

  std::cout << "\nFinished Experiment 1." << std::endl;
  std::cout << "Saved results to " << result_dir << std::endl;
  std::cout << "CSV summary: " << result_dir << "subgoal_k_sweep.csv" << std::endl;

  if (!at_least_one_success)
  {
    CONSOLE_BRIDGE_logWarn("No K produced a collision-free optimized trajectory according to the external checker.");
  }

  return true;
}

}  // namespace tesseract_examples