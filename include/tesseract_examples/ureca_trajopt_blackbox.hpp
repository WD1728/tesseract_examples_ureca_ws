#ifndef TESSERACT_EXAMPLES_URECA_TRAJOPT_BLACKBOX_HPP
#define TESSERACT_EXAMPLES_URECA_TRAJOPT_BLACKBOX_HPP

#include <tesseract_examples/ureca_experiment_config.hpp>

#include <tesseract_environment/fwd.h>

#include <Eigen/Core>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tesseract_examples
{
struct UrecaTrajOptRequest
{
  std::shared_ptr<tesseract_environment::Environment> environment;
  UrecaSceneConfig scene;
  Eigen::VectorXd q_start;
  Eigen::VectorXd q_goal;
  std::optional<Eigen::VectorXd> q_subgoal;
  int subgoal_step{ -1 };
  int n_steps{ 0 };
  std::vector<Eigen::VectorXd> seed;
};

struct TrajOptResult
{
  bool success{ false };
  std::string status_message;
  double objective{ 0.0 };
  double solve_time_ms{ 0.0 };
  double seed_min_clearance{ 0.0 };
  double opt_min_clearance{ 0.0 };
  std::vector<Eigen::VectorXd> seed_trajectory;
  std::vector<Eigen::VectorXd> optimized_trajectory;
};

TrajOptResult runTrajOptBlackBox(const UrecaTrajOptRequest& request);

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_TRAJOPT_BLACKBOX_HPP
