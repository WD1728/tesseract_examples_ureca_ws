#ifndef TESSERACT_EXAMPLES_URECA_EXPERIMENT_UTILS_HPP
#define TESSERACT_EXAMPLES_URECA_EXPERIMENT_UTILS_HPP

#include <tesseract_examples/ureca_experiment_config.hpp>
#include <tesseract_examples/ureca_trajopt_blackbox.hpp>

#include <tesseract_environment/fwd.h>

#include <Eigen/Core>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tesseract_examples
{
struct UrecaExperimentContext
{
  std::filesystem::path workspace_root;
  std::filesystem::path output_dir;
  std::filesystem::path runtime_dir;
  UrecaSuiteConfig suite_config;
};

std::shared_ptr<tesseract_environment::Environment> createEnvironmentForScene(const UrecaSceneConfig& scene,
                                                                              const std::filesystem::path& runtime_dir,
                                                                              std::string& status_message);
std::vector<Eigen::VectorXd> makeLinearSeed(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, int n_steps);
std::vector<Eigen::VectorXd> makeSubgoalSeed(const Eigen::VectorXd& start,
                                             const Eigen::VectorXd& subgoal,
                                             const Eigen::VectorXd& goal,
                                             int n_steps,
                                             int subgoal_step);
std::vector<Eigen::VectorXd> perturbSeed(const std::vector<Eigen::VectorXd>& seed,
                                         int axis,
                                         double perturbation);
UrecaSummaryRow makeSummaryRow(const std::string& experiment_id,
                               const std::string& case_id,
                               const UrecaSceneConfig& scene,
                               int k,
                               const std::string& seed_type,
                               double perturbation,
                               double obstacle_offset,
                               const std::optional<Eigen::VectorXd>& subgoal,
                               const TrajOptResult& result,
                               const std::string& trajectory_file);
std::filesystem::path ensureExperimentSubdir(const UrecaExperimentContext& context, const std::string& experiment_id);
std::vector<std::string> splitCsvList(const std::string& value);

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_EXPERIMENT_UTILS_HPP
