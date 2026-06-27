#include <algorithm>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_two_robot_two_subgoal_example.hpp>
#include <tesseract_examples/ureca_result_utils.hpp>

#include <cmath>
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
constexpr double ROBOT_RADIUS = 0.10;
constexpr double VELOCITY_COEFF = 0.01;
constexpr double ACCELERATION_COEFF = 0.01;

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

Eigen::Vector3d interpolate3(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double alpha)
{
  return a + alpha * (b - a);
}

Eigen::Vector3d seedPointForRobot(int i,
                                  int k,
                                  int n,
                                  const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& subgoal,
                                  const Eigen::Vector3d& goal)
{
  if (i <= k)
    return interpolate3(start, subgoal, static_cast<double>(i) / static_cast<double>(k));
  return interpolate3(subgoal, goal, static_cast<double>(i - k) / static_cast<double>(n - 1 - k));
}

std::vector<Eigen::VectorXd> buildSeed(int n,
                                       int k1,
                                       int k2,
                                       const Eigen::Vector3d& start1,
                                       const Eigen::Vector3d& subgoal1,
                                       const Eigen::Vector3d& goal1,
                                       const Eigen::Vector3d& start2,
                                       const Eigen::Vector3d& subgoal2,
                                       const Eigen::Vector3d& goal2)
{
  std::vector<Eigen::VectorXd> seed;
  seed.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    Eigen::VectorXd q(6);
    const Eigen::Vector3d r1 = seedPointForRobot(i, k1, n, start1, subgoal1, goal1);
    const Eigen::Vector3d r2 = seedPointForRobot(i, k2, n, start2, subgoal2, goal2);
    q << r1.x(), r1.y(), r1.z(), r2.x(), r2.y(), r2.z();
    seed.push_back(q);
  }
  return seed;
}

CompositeInstruction buildProgram(const std::vector<std::string>& joint_names,
                                  const std::vector<Eigen::VectorXd>& seed,
                                  int k1,
                                  int k2)
{
  CompositeInstruction program("ureca_two_robot_two_subgoal_program", ManipulatorInfo("manipulator"));
  const int n = static_cast<int>(seed.size());
  for (int i = 0; i < n; ++i)
  {
    std::string profile = "seed_profile";
    if (i == 0 || i == n - 1)
      profile = "fixed_both_profile";
    else if (i == k1 && i == k2)
      profile = "fixed_both_profile";
    else if (i == k1)
      profile = "fixed_r1_profile";
    else if (i == k2)
      profile = "fixed_r2_profile";

    StateWaypoint wp{ joint_names, seed[static_cast<std::size_t>(i)] };
    MoveInstruction move(wp, MoveInstructionType::FREESPACE, profile);
    program.push_back(move);
  }
  return program;
}

std::shared_ptr<tesseract_common::ProfileDictionary> createProfiles(bool obstacle_enabled)
{
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.0, 1000);
  composite_profile->collision_cost_config.enabled = obstacle_enabled;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->collision_constraint_config = trajopt_common::TrajOptCollisionConfig(0.0, 20);
  composite_profile->collision_constraint_config.enabled = obstacle_enabled;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(6) * VELOCITY_COEFF;
  composite_profile->smooth_accelerations = true;
  composite_profile->acceleration_coeff = Eigen::VectorXd::Ones(6) * ACCELERATION_COEFF;
  composite_profile->smooth_jerks = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "ureca_two_robot_two_subgoal_program", composite_profile);

  auto fixed_both = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_both->cartesian_cost_config.enabled = false;
  fixed_both->cartesian_constraint_config.enabled = false;
  fixed_both->joint_cost_config.enabled = false;
  fixed_both->joint_constraint_config.enabled = true;
  fixed_both->joint_constraint_config.coeff = Eigen::VectorXd::Ones(6);
  profiles->addProfile(TRAJOPT_NAMESPACE, "fixed_both_profile", fixed_both);

  auto fixed_r1 = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_r1->cartesian_cost_config.enabled = false;
  fixed_r1->cartesian_constraint_config.enabled = false;
  fixed_r1->joint_cost_config.enabled = false;
  fixed_r1->joint_constraint_config.enabled = true;
  fixed_r1->joint_constraint_config.coeff = Eigen::VectorXd::Zero(6);
  fixed_r1->joint_constraint_config.coeff.head(3).setOnes();
  profiles->addProfile(TRAJOPT_NAMESPACE, "fixed_r1_profile", fixed_r1);

  auto fixed_r2 = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_r2->cartesian_cost_config.enabled = false;
  fixed_r2->cartesian_constraint_config.enabled = false;
  fixed_r2->joint_cost_config.enabled = false;
  fixed_r2->joint_constraint_config.enabled = true;
  fixed_r2->joint_constraint_config.coeff = Eigen::VectorXd::Zero(6);
  fixed_r2->joint_constraint_config.coeff.tail(3).setOnes();
  profiles->addProfile(TRAJOPT_NAMESPACE, "fixed_r2_profile", fixed_r2);

  auto seed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  seed_profile->cartesian_cost_config.enabled = false;
  seed_profile->cartesian_constraint_config.enabled = false;
  seed_profile->joint_cost_config.enabled = false;
  seed_profile->joint_constraint_config.enabled = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "seed_profile", seed_profile);
  return profiles;
}

bool runSweep(std::shared_ptr<tesseract_environment::Environment> env,
              const std::filesystem::path& result_dir,
              bool obstacle_enabled)
{
  const int n = 10;
  const std::vector<std::string> joint_names{
    "r1_joint_x", "r1_joint_y", "r1_joint_z", "r2_joint_x", "r2_joint_y", "r2_joint_z"
  };

  const Eigen::Vector3d start1(0.0, 0.0, 0.0);
  const Eigen::Vector3d subgoal1(1.0, 1.2, 0.0);
  const Eigen::Vector3d goal1(2.0, 1.0, 0.0);
  const Eigen::Vector3d start2(2.0, 0.0, 0.0);
  const Eigen::Vector3d subgoal2(1.0, -0.2, 0.0);
  const Eigen::Vector3d goal2(0.0, 1.0, 0.0);

  Eigen::VectorXd q_start(6);
  q_start << start1.x(), start1.y(), start1.z(), start2.x(), start2.y(), start2.z();
  env->setState(joint_names, q_start);
  if (!env->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
  {
    CONSOLE_BRIDGE_logError("Failed to set BulletDiscreteBVHManager.");
    return false;
  }

  urecaEnsureResultLayout(result_dir);
  auto profiles = createProfiles(obstacle_enabled);
  TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);
  std::vector<UrecaSummaryRow> rows;

  const Eigen::Vector3d obstacle_center(1.0, 0.4, 0.0);
  const double obstacle_radius = obstacle_enabled ? 0.25 : 0.0;

  for (int k1 = 1; k1 <= 8; ++k1)
  {
    for (int k2 = 1; k2 <= 8; ++k2)
    {
      const std::vector<Eigen::VectorXd> seed = buildSeed(n, k1, k2, start1, subgoal1, goal1, start2, subgoal2, goal2);
      CompositeInstruction program = buildProgram(joint_names, seed, k1, k2);

      PlannerRequest request;
      request.instructions = program;
      request.env = env;
      request.profiles = profiles;
      request.verbose = false;
      request.format_result_as_input = false;

      tesseract_common::Stopwatch stopwatch;
      stopwatch.start();
      PlannerResponse response = planner.solve(request);
      stopwatch.stop();

      UrecaSummaryRow row;
      row.case_id = "k1_" + std::to_string(k1) + "_k2_" + std::to_string(k2);
      row.k1 = k1;
      row.k2 = k2;
      row.trajopt_success = response.successful;
      row.trajopt_message = response.message;
      row.solve_time_ms = stopwatch.elapsedSeconds() * 1000.0;

      if (response.successful)
      {
        std::vector<Eigen::VectorXd> trajectory;
        if (extractTrajectory(response.results, trajectory))
        {
          const double min_obstacle_r1 = obstacle_enabled
                                             ? urecaComputeMinSphereObstacleClearanceProxy(
                                                   trajectory, obstacle_center, obstacle_radius, ROBOT_RADIUS, 0, 3)
                                             : std::numeric_limits<double>::infinity();
          const double min_obstacle_r2 = obstacle_enabled
                                             ? urecaComputeMinSphereObstacleClearanceProxy(
                                                   trajectory, obstacle_center, obstacle_radius, ROBOT_RADIUS, 3, 3)
                                             : std::numeric_limits<double>::infinity();
          const double min_obstacle = std::min(min_obstacle_r1, min_obstacle_r2);
          const double min_inter_robot = urecaComputeMinInterRobotClearanceProxy(trajectory, ROBOT_RADIUS);
          row.post_min_obstacle_clearance_proxy = obstacle_enabled ? min_obstacle : std::numeric_limits<double>::quiet_NaN();
          row.post_min_inter_robot_clearance_proxy = min_inter_robot;
          row.post_min_clearance_proxy = obstacle_enabled ? std::min(min_obstacle, min_inter_robot) : min_inter_robot;
          row.collision_free = row.post_min_clearance_proxy >= 0.0;
          row.opt_path_objective = urecaComputePathObjective(trajectory, VELOCITY_COEFF);
          row.post_path_length = urecaComputePathLength(trajectory);
          row.post_smoothness_proxy = urecaComputeSmoothnessProxy(trajectory);
          row.trajectory_file = row.case_id + ".csv";

          const auto obstacle_per_step = obstacle_enabled
                                             ? [&trajectory, &obstacle_center, obstacle_radius]() {
                                                 auto r1 = urecaComputePerStepObstacleClearanceProxy(
                                                     trajectory, obstacle_center, obstacle_radius, ROBOT_RADIUS, 0, 3);
                                                 auto r2 = urecaComputePerStepObstacleClearanceProxy(
                                                     trajectory, obstacle_center, obstacle_radius, ROBOT_RADIUS, 3, 3);
                                                 for (std::size_t i = 0; i < r1.size(); ++i)
                                                   r1[i] = std::min(r1[i], r2[i]);
                                                 return r1;
                                               }()
                                             : std::vector<double>(trajectory.size(), std::numeric_limits<double>::quiet_NaN());
          const auto inter_robot_per_step = urecaComputePerStepInterRobotClearanceProxy(trajectory, ROBOT_RADIUS);
          urecaWriteTwoRobotTrajectoryCsv(result_dir / "trajectories" / row.trajectory_file,
                                          trajectory,
                                          obstacle_per_step,
                                          inter_robot_per_step);
        }
      }

      rows.push_back(row);
    }
  }

  urecaWriteSummaryCsv(result_dir / "summary.csv", rows);
  return true;
}

}  // namespace

UrecaTwoRobotTwoSubgoalExample::UrecaTwoRobotTwoSubgoalExample(std::shared_ptr<tesseract_environment::Environment> env,
                                                               std::shared_ptr<tesseract_visualization::Visualization> plotter,
                                                               bool ifopt,
                                                               bool debug)
  : Example(std::move(env), std::move(plotter))
  , ifopt_(ifopt)
  , debug_(debug)
{
}

bool UrecaTwoRobotTwoSubgoalExample::run()
{
  console_bridge::setLogLevel(debug_ ? console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG :
                                       console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
  return runSweep(env_, "/home/wenda/tesseract_ws/result_ureca/two_robot_two_subgoal", false);
}

}  // namespace tesseract_examples
