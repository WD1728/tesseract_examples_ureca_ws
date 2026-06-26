#ifndef TESSERACT_EXAMPLES_URECA_EXPERIMENT_CONFIG_HPP
#define TESSERACT_EXAMPLES_URECA_EXPERIMENT_CONFIG_HPP

#include <Eigen/Core>

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace tesseract_examples
{
struct UrecaSceneConfig
{
  std::string scene_id;
  int dim{ 0 };
  int n_steps{ 25 };
  std::vector<std::string> joint_names;
  std::string manipulator{ "manipulator" };
  std::string base_link{ "base_link" };
  std::string tip_link{ "robot_body" };
  std::filesystem::path urdf_template;
  std::filesystem::path srdf_template;
  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  std::optional<Eigen::VectorXd> subgoal;
  Eigen::Vector3d obstacle_center{ 0.0, 0.0, 0.0 };
  double obstacle_radius{ 0.20 };
  double robot_radius{ 0.10 };
  double safe_margin{ 0.0 };
  double collision_cost_margin{ 0.0 };
  double collision_cost_weight{ 1000.0 };
  bool collision_constraint_enabled{ false };
  double collision_constraint_margin{ 0.0 };
  double collision_constraint_weight{ 20.0 };
  double velocity_coeff{ 0.01 };
};

struct UrecaSuiteConfig
{
  int default_n_steps_2d{ 25 };
  int default_n_steps_3d{ 25 };
  Eigen::Vector2d two_d_start{ 0.0, 0.0 };
  Eigen::Vector2d two_d_goal{ 2.0, 1.0 };
  Eigen::Vector2d two_d_subgoal{ 1.0, 1.05 };
  Eigen::Vector2d two_d_obstacle_center{ 1.0, 0.6 };
  double two_d_obstacle_radius{ 0.20 };
  double two_d_robot_radius{ 0.10 };
  double two_d_velocity_coeff{ 0.01 };
  double two_d_collision_cost_margin{ 0.0025 };
  double two_d_collision_cost_weight{ 1000.0 };
  bool two_d_collision_constraint_enabled{ false };
  double two_d_collision_constraint_margin{ 0.0 };
  double two_d_collision_constraint_weight{ 20.0 };

  Eigen::Vector3d three_d_start{ 0.0, 0.0, 0.0 };
  Eigen::Vector3d three_d_goal{ 2.0, 1.0, 0.0 };
  Eigen::Vector3d three_d_subgoal{ 1.0, 1.05, 0.0 };
  Eigen::Vector3d three_d_obstacle_center{ 0.5, 0.6, 0.0 };
  double three_d_obstacle_radius{ 0.20 };
  double three_d_robot_radius{ 0.10 };
  double three_d_velocity_coeff{ 0.01 };
  double three_d_collision_cost_margin{ 0.02 };
  double three_d_collision_cost_weight{ 1000.0 };
  bool three_d_collision_constraint_enabled{ true };
  double three_d_collision_constraint_margin{ 0.0 };
  double three_d_collision_constraint_weight{ 20.0 };

  int default_selected_k_2d{ 12 };
  int default_selected_k_3d{ 12 };
  int default_early_k{ 2 };
  int default_late_k_offset{ 2 };
  int blackbox_demo_k{ 12 };

  std::vector<double> penetration_targets{ -0.02, -0.05, -0.10, -0.20, -0.30 };
  std::vector<double> symmetry_perturbations{ 0.00, 0.01, -0.01, 0.03, -0.03, 0.05, -0.05 };
  std::vector<double> obstacle_radius_values{ 0.15, 0.20, 0.25, 0.30 };
  std::vector<double> safe_margin_values{ 0.00, 0.05, 0.10 };
  std::vector<double> two_d_subgoal_y_values{ 0.9, 1.0, 1.1, 1.2, 1.3 };
  std::vector<double> three_d_subgoal_z_values{ -0.1, -0.05, 0.0, 0.05, 0.1 };
};

struct UrecaSummaryRow
{
  std::string experiment_id;
  std::string case_id;
  int dim{ 0 };
  int N{ 0 };
  int K{ -1 };
  std::string seed_type;
  double perturbation{ 0.0 };
  double obstacle_radius{ 0.0 };
  double safe_radius{ 0.0 };
  double obstacle_offset{ 0.0 };
  double subgoal_x{ 0.0 };
  double subgoal_y{ 0.0 };
  double subgoal_z{ 0.0 };
  double seed_min_clearance{ 0.0 };
  double opt_min_clearance{ 0.0 };
  double max_penetration{ 0.0 };
  double objective{ 0.0 };
  double path_length{ 0.0 };
  double smoothness{ 0.0 };
  double solve_time_ms{ 0.0 };
  std::string status;
  bool success{ false };
  std::string trajectory_file;
};

UrecaSuiteConfig loadUrecaSuiteConfig(const std::filesystem::path& config_path);
std::vector<std::string> supportedUrecaExperimentIds();
UrecaSceneConfig makeBase2DScene(const UrecaSuiteConfig& config, const std::filesystem::path& workspace_root);
UrecaSceneConfig makeBase3DScene(const UrecaSuiteConfig& config, const std::filesystem::path& workspace_root);
int clampSubgoalStep(int requested_k, int n_steps);
std::map<std::string, std::string> parseFlatConfig(const std::filesystem::path& config_path);

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_EXPERIMENT_CONFIG_HPP
