#ifndef TESSERACT_EXAMPLES_URECA_RESULT_UTILS_HPP
#define TESSERACT_EXAMPLES_URECA_RESULT_UTILS_HPP

#include <Eigen/Core>

#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace tesseract_examples
{
struct UrecaSummaryRow
{
  std::string case_id;
  int K{ -1 };
  int k1{ -1 };
  int k2{ -1 };
  double perturbation{ 0.0 };
  std::string penetration_label;
  bool trajopt_success{ false };
  std::string trajopt_message;
  double solve_time_ms{ std::numeric_limits<double>::quiet_NaN() };
  bool collision_free{ false };
  double opt_path_objective{ std::numeric_limits<double>::quiet_NaN() };
  double post_path_length{ std::numeric_limits<double>::quiet_NaN() };
  double post_smoothness_proxy{ std::numeric_limits<double>::quiet_NaN() };
  double post_min_clearance_proxy{ std::numeric_limits<double>::quiet_NaN() };
  double post_min_obstacle_clearance_proxy{ std::numeric_limits<double>::quiet_NaN() };
  double post_min_inter_robot_clearance_proxy{ std::numeric_limits<double>::quiet_NaN() };
  std::string trajectory_file;
};

double urecaComputePathObjective(const std::vector<Eigen::VectorXd>& trajectory, double velocity_coeff);
double urecaComputePathLength(const std::vector<Eigen::VectorXd>& trajectory);
double urecaComputeSmoothnessProxy(const std::vector<Eigen::VectorXd>& trajectory);
double urecaComputeMinSphereObstacleClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                   const Eigen::Vector3d& obstacle_center,
                                                   double obstacle_radius,
                                                   double body_radius,
                                                   int offset,
                                                   int dim);
double urecaComputeMinInterRobotClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                               double body_radius);
std::vector<double> urecaComputePerStepObstacleClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                              const Eigen::Vector3d& obstacle_center,
                                                              double obstacle_radius,
                                                              double body_radius,
                                                              int offset,
                                                              int dim);
std::vector<double> urecaComputePerStepInterRobotClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                                double body_radius);
void urecaEnsureResultLayout(const std::filesystem::path& result_dir);
void urecaWriteSummaryCsv(const std::filesystem::path& csv_path, const std::vector<UrecaSummaryRow>& rows);
void urecaWriteSingleRobotTrajectoryCsv(const std::filesystem::path& csv_path,
                                        const std::vector<Eigen::VectorXd>& trajectory,
                                        const std::vector<double>& clearance_proxy,
                                        int dim);
void urecaWriteTwoRobotTrajectoryCsv(const std::filesystem::path& csv_path,
                                     const std::vector<Eigen::VectorXd>& trajectory,
                                     const std::vector<double>& obstacle_clearance_proxy,
                                     const std::vector<double>& inter_robot_clearance_proxy);

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_RESULT_UTILS_HPP
