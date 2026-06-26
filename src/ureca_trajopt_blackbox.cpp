#include <tesseract_examples/ureca_trajopt_blackbox.hpp>

#include <tesseract_examples/ureca_experiment_metrics.hpp>

#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/stopwatch.h>

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

#include <stdexcept>

using namespace tesseract_planning;

namespace tesseract_examples
{
namespace
{
static const std::string TRAJOPT_NAMESPACE = "TrajOptMotionPlannerTask";

bool extractTrajectory(const CompositeInstruction& result, std::vector<Eigen::VectorXd>& trajectory)
{
  trajectory.clear();
  auto flattened = result.flatten();

  for (const auto& instruction_ref : flattened)
  {
    const auto& instruction = instruction_ref.get();
    if (!instruction.isMoveInstruction())
      continue;

    const auto& move = instruction.as<MoveInstructionPoly>();
    const auto& waypoint = move.getWaypoint();
    if (!waypoint.isStateWaypoint())
      continue;

    trajectory.push_back(waypoint.as<StateWaypointPoly>().getPosition());
  }

  return !trajectory.empty();
}

std::shared_ptr<tesseract_common::ProfileDictionary> createProfiles(const UrecaSceneConfig& scene)
{
  auto profiles = std::make_shared<tesseract_common::ProfileDictionary>();

  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config =
      trajopt_common::TrajOptCollisionConfig(scene.collision_cost_margin, scene.collision_cost_weight);
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  composite_profile->collision_constraint_config =
      trajopt_common::TrajOptCollisionConfig(scene.collision_constraint_margin, scene.collision_constraint_weight);
  composite_profile->collision_constraint_config.enabled = scene.collision_constraint_enabled;
  composite_profile->collision_constraint_config.collision_check_config.type =
      tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;

  composite_profile->smooth_velocities = true;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(scene.dim) * scene.velocity_coeff;
  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;

  profiles->addProfile(TRAJOPT_NAMESPACE, "ureca_program", composite_profile);

  auto fixed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  fixed_profile->cartesian_cost_config.enabled = false;
  fixed_profile->cartesian_constraint_config.enabled = false;
  fixed_profile->joint_cost_config.enabled = false;
  fixed_profile->joint_constraint_config.enabled = true;
  fixed_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(scene.dim);
  profiles->addProfile(TRAJOPT_NAMESPACE, "fixed_profile", fixed_profile);

  auto seed_profile = std::make_shared<TrajOptDefaultMoveProfile>();
  seed_profile->cartesian_cost_config.enabled = false;
  seed_profile->cartesian_constraint_config.enabled = false;
  seed_profile->joint_cost_config.enabled = false;
  seed_profile->joint_constraint_config.enabled = false;
  profiles->addProfile(TRAJOPT_NAMESPACE, "seed_profile", seed_profile);

  return profiles;
}

CompositeInstruction buildProgram(const UrecaTrajOptRequest& request)
{
  if (request.n_steps < 2)
    throw std::runtime_error("URECA request requires at least two steps.");
  if (static_cast<int>(request.q_start.size()) != request.scene.dim ||
      static_cast<int>(request.q_goal.size()) != request.scene.dim)
    throw std::runtime_error("URECA request dimension does not match start/goal.");
  if (static_cast<int>(request.seed.size()) != request.n_steps)
    throw std::runtime_error("URECA request seed size does not match n_steps.");

  CompositeInstruction program("ureca_program",
                               tesseract_common::ManipulatorInfo(request.scene.manipulator,
                                                                 request.scene.base_link,
                                                                 request.scene.tip_link));

  const int k = request.q_subgoal.has_value() ? clampSubgoalStep(request.subgoal_step, request.n_steps) : -1;

  for (int i = 0; i < request.n_steps; ++i)
  {
    const bool is_fixed = (i == 0) || (i == request.n_steps - 1) || (request.q_subgoal.has_value() && i == k);
    StateWaypoint waypoint(request.scene.joint_names, request.seed[static_cast<std::size_t>(i)]);
    MoveInstruction instruction(waypoint,
                                MoveInstructionType::FREESPACE,
                                is_fixed ? "fixed_profile" : "seed_profile");

    if (i == 0)
      instruction.setDescription("start");
    else if (i == request.n_steps - 1)
      instruction.setDescription("goal");
    else if (request.q_subgoal.has_value() && i == k)
      instruction.setDescription("subgoal");
    else
      instruction.setDescription("seed_" + std::to_string(i));

    program.push_back(instruction);
  }

  return program;
}

void enforceFixedWaypoints(UrecaTrajOptRequest& request)
{
  request.seed.front() = request.q_start;
  request.seed.back() = request.q_goal;
  if (request.q_subgoal.has_value())
  {
    const int k = clampSubgoalStep(request.subgoal_step, request.n_steps);
    request.seed[static_cast<std::size_t>(k)] = *request.q_subgoal;
  }
}

}  // namespace

TrajOptResult runTrajOptBlackBox(const UrecaTrajOptRequest& input_request)
{
  UrecaTrajOptRequest request = input_request;
  TrajOptResult result;
  result.status_message = "not_started";

  try
  {
    if (request.environment == nullptr)
      throw std::runtime_error("URECA black-box request is missing an environment.");

    enforceFixedWaypoints(request);
    result.seed_trajectory = request.seed;
    result.seed_min_clearance = computeUrecaMinClearance(request.scene, result.seed_trajectory);

    auto profiles = createProfiles(request.scene);
    CompositeInstruction program = buildProgram(request);
    TrajOptMotionPlanner planner(TRAJOPT_NAMESPACE);

    PlannerRequest planner_request;
    planner_request.instructions = program;
    planner_request.env = request.environment;
    planner_request.profiles = profiles;
    planner_request.verbose = false;
    planner_request.format_result_as_input = false;

    tesseract_common::Stopwatch stopwatch;
    stopwatch.start();
    PlannerResponse response = planner.solve(planner_request);
    stopwatch.stop();

    result.solve_time_ms = stopwatch.elapsedSeconds() * 1000.0;
    result.status_message = response.message;
    result.success = response.successful;

    if (!response.successful)
      return result;

    if (!extractTrajectory(response.results, result.optimized_trajectory))
      throw std::runtime_error("Failed to extract optimized trajectory from TrajOpt response.");

    result.opt_min_clearance = computeUrecaMinClearance(request.scene, result.optimized_trajectory);
    result.objective = computeUrecaObjective(request.scene, result.optimized_trajectory);
    result.success = true;
    if (result.status_message.empty())
      result.status_message = "success";
    return result;
  }
  catch (const std::exception& e)
  {
    result.success = false;
    result.status_message = e.what();
    return result;
  }
}

}  // namespace tesseract_examples
