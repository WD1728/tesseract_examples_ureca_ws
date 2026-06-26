#include <tesseract_examples/ureca_experiment_metrics.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace tesseract_examples
{
namespace
{
double pointSegmentDistance(const Eigen::VectorXd& point,
                            const Eigen::VectorXd& a,
                            const Eigen::VectorXd& b)
{
  const Eigen::VectorXd ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom < 1e-12)
    return (point - a).norm();

  double t = (point - a).dot(ab) / denom;
  t = std::max(0.0, std::min(1.0, t));
  return (point - (a + t * ab)).norm();
}

std::string formatNumber(double value)
{
  if (std::isnan(value))
    return "nan";

  std::ostringstream ss;
  ss << std::setprecision(10) << value;
  return ss.str();
}

}  // namespace

double computeUrecaPathLength(const std::vector<Eigen::VectorXd>& trajectory)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i)
    total += (trajectory[i + 1] - trajectory[i]).norm();
  return total;
}

double computeUrecaSmoothness(const std::vector<Eigen::VectorXd>& trajectory)
{
  double total = 0.0;
  for (std::size_t i = 1; i + 1 < trajectory.size(); ++i)
    total += (trajectory[i + 1] - 2.0 * trajectory[i] + trajectory[i - 1]).squaredNorm();
  return total;
}

double computeUrecaMinClearance(const UrecaSceneConfig& scene, const std::vector<Eigen::VectorXd>& trajectory)
{
  if (trajectory.empty())
    return std::numeric_limits<double>::quiet_NaN();

  Eigen::VectorXd obstacle(scene.dim);
  obstacle.setZero();
  for (int i = 0; i < scene.dim; ++i)
    obstacle(i) = scene.obstacle_center(i);

  double min_distance = std::numeric_limits<double>::infinity();

  for (const auto& point : trajectory)
    min_distance = std::min(min_distance, (point - obstacle).norm());

  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i)
    min_distance = std::min(min_distance, pointSegmentDistance(obstacle, trajectory[i], trajectory[i + 1]));

  return min_distance - (scene.obstacle_radius + scene.robot_radius + scene.safe_margin);
}

double computeUrecaObjective(const UrecaSceneConfig& scene, const std::vector<Eigen::VectorXd>& trajectory)
{
  double objective = 0.0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i)
    objective += scene.velocity_coeff * (trajectory[i + 1] - trajectory[i]).squaredNorm();
  return objective;
}

double computeUrecaMaxPenetration(double min_clearance)
{
  return std::max(0.0, -min_clearance);
}

std::vector<double> computePerWaypointClearance(const UrecaSceneConfig& scene,
                                                const std::vector<Eigen::VectorXd>& trajectory)
{
  std::vector<double> clearance;
  clearance.reserve(trajectory.size());

  Eigen::VectorXd obstacle(scene.dim);
  obstacle.setZero();
  for (int i = 0; i < scene.dim; ++i)
    obstacle(i) = scene.obstacle_center(i);

  for (const auto& point : trajectory)
    clearance.push_back((point - obstacle).norm() - (scene.obstacle_radius + scene.robot_radius + scene.safe_margin));

  return clearance;
}

void writeUrecaSummaryCsv(const std::filesystem::path& csv_path, const std::vector<UrecaSummaryRow>& rows)
{
  std::ofstream output(csv_path);
  output << "experiment_id,case_id,dim,N,K,seed_type,perturbation,obstacle_radius,safe_radius,obstacle_offset,"
            "subgoal_x,subgoal_y,subgoal_z,seed_min_clearance,opt_min_clearance,max_penetration,objective,"
            "path_length,smoothness,solve_time_ms,status,success,trajectory_file\n";

  for (const auto& row : rows)
  {
    output << row.experiment_id << "," << row.case_id << "," << row.dim << "," << row.N << "," << row.K << ","
           << row.seed_type << "," << formatNumber(row.perturbation) << "," << formatNumber(row.obstacle_radius) << ","
           << formatNumber(row.safe_radius) << "," << formatNumber(row.obstacle_offset) << "," << formatNumber(row.subgoal_x) << ","
           << formatNumber(row.subgoal_y) << "," << formatNumber(row.subgoal_z) << "," << formatNumber(row.seed_min_clearance) << ","
           << formatNumber(row.opt_min_clearance) << "," << formatNumber(row.max_penetration) << "," << formatNumber(row.objective) << ","
           << formatNumber(row.path_length) << "," << formatNumber(row.smoothness) << "," << formatNumber(row.solve_time_ms) << ","
           << row.status << "," << (row.success ? 1 : 0) << "," << row.trajectory_file << "\n";
  }
}

void writeUrecaTrajectoryCsv(const std::filesystem::path& csv_path,
                             const UrecaSceneConfig& scene,
                             const std::vector<Eigen::VectorXd>& trajectory)
{
  const std::vector<double> per_waypoint_clearance = computePerWaypointClearance(scene, trajectory);

  std::ofstream output(csv_path);
  output << "step,q0,q1,q2,x,y,z,clearance\n";

  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    const auto& q = trajectory[i];
    const double q0 = q.size() > 0 ? q(0) : 0.0;
    const double q1 = q.size() > 1 ? q(1) : 0.0;
    const double q2 = q.size() > 2 ? q(2) : 0.0;
    output << i << "," << formatNumber(q0) << "," << formatNumber(q1) << "," << formatNumber(q2) << ","
           << formatNumber(q0) << "," << formatNumber(q1) << "," << formatNumber(q2) << ","
           << formatNumber(per_waypoint_clearance[i]) << "\n";
  }
}

int findBestSuccessfulK(const std::filesystem::path& summary_csv)
{
  std::ifstream input(summary_csv);
  if (!input.is_open())
    return -1;

  std::string line;
  if (!std::getline(input, line))
    return -1;

  int best_k = -1;
  double best_objective = std::numeric_limits<double>::infinity();
  while (std::getline(input, line))
  {
    std::stringstream ss(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(ss, field, ','))
      fields.push_back(field);

    if (fields.size() < 23)
      continue;
    const bool success = (fields[21] == "1" || fields[21] == "true" || fields[21] == "True");
    if (!success)
      continue;

    const int k = std::stoi(fields[4]);
    const double objective = std::stod(fields[16]);
    if (objective < best_objective)
    {
      best_objective = objective;
      best_k = k;
    }
  }

  return best_k;
}

std::string formatOptionalCoordinate(const std::optional<Eigen::VectorXd>& subgoal, int index)
{
  if (!subgoal.has_value() || index < 0 || index >= subgoal->size())
    return "none";
  return formatNumber((*subgoal)(index));
}

}  // namespace tesseract_examples
