#ifndef TESSERACT_EXAMPLES_URECA_EXPERIMENT_METRICS_HPP
#define TESSERACT_EXAMPLES_URECA_EXPERIMENT_METRICS_HPP

#include <tesseract_examples/ureca_experiment_config.hpp>

#include <Eigen/Core>

#include <filesystem>
#include <string>
#include <vector>

namespace tesseract_examples
{
double computeUrecaPathLength(const std::vector<Eigen::VectorXd>& trajectory);
double computeUrecaSmoothness(const std::vector<Eigen::VectorXd>& trajectory);
double computeUrecaMinClearance(const UrecaSceneConfig& scene, const std::vector<Eigen::VectorXd>& trajectory);
double computeUrecaObjective(const UrecaSceneConfig& scene, const std::vector<Eigen::VectorXd>& trajectory);
double computeUrecaMaxPenetration(double min_clearance);
std::vector<double> computePerWaypointClearance(const UrecaSceneConfig& scene,
                                                const std::vector<Eigen::VectorXd>& trajectory);
void writeUrecaSummaryCsv(const std::filesystem::path& csv_path, const std::vector<UrecaSummaryRow>& rows);
void writeUrecaTrajectoryCsv(const std::filesystem::path& csv_path,
                             const UrecaSceneConfig& scene,
                             const std::vector<Eigen::VectorXd>& trajectory);
int findBestSuccessfulK(const std::filesystem::path& summary_csv);
std::string formatOptionalCoordinate(const std::optional<Eigen::VectorXd>& subgoal, int index);

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_EXPERIMENT_METRICS_HPP
