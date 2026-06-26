#include <tesseract_examples/ureca_result_utils.hpp>

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
double pointSegmentDistance(const Eigen::VectorXd& p,
                            const Eigen::VectorXd& a,
                            const Eigen::VectorXd& b)
{
  const Eigen::VectorXd ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom < 1e-12)
    return (p - a).norm();

  double t = (p - a).dot(ab) / denom;
  t = std::max(0.0, std::min(1.0, t));
  return (p - (a + t * ab)).norm();
}

std::string formatDouble(double value)
{
  if (std::isnan(value))
    return "nan";

  std::ostringstream ss;
  ss << std::setprecision(10) << value;
  return ss.str();
}

std::string escapeCsv(const std::string& value)
{
  if (value.find_first_of(",\"\n") == std::string::npos)
    return value;

  std::string escaped = "\"";
  for (char c : value)
  {
    if (c == '"')
      escaped += "\"\"";
    else
      escaped += c;
  }
  escaped += "\"";
  return escaped;
}

Eigen::VectorXd slicePoint(const Eigen::VectorXd& q, int offset, int dim)
{
  Eigen::VectorXd out(dim);
  for (int i = 0; i < dim; ++i)
    out(i) = q(offset + i);
  return out;
}

}  // namespace

double urecaComputePathLength(const std::vector<Eigen::VectorXd>& trajectory)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i)
    total += (trajectory[i + 1] - trajectory[i]).norm();
  return total;
}

double urecaComputeSmoothnessProxy(const std::vector<Eigen::VectorXd>& trajectory)
{
  double total = 0.0;
  for (std::size_t i = 1; i + 1 < trajectory.size(); ++i)
    total += (trajectory[i + 1] - 2.0 * trajectory[i] + trajectory[i - 1]).squaredNorm();
  return total;
}

double urecaComputeMinSphereObstacleClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                   const Eigen::Vector3d& obstacle_center,
                                                   double obstacle_radius,
                                                   double body_radius,
                                                   int offset,
                                                   int dim)
{
  if (trajectory.empty())
    return std::numeric_limits<double>::quiet_NaN();

  Eigen::VectorXd center(dim);
  for (int i = 0; i < dim; ++i)
    center(i) = obstacle_center(i);

  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& q : trajectory)
    min_dist = std::min(min_dist, (slicePoint(q, offset, dim) - center).norm());

  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i)
  {
    const Eigen::VectorXd a = slicePoint(trajectory[i], offset, dim);
    const Eigen::VectorXd b = slicePoint(trajectory[i + 1], offset, dim);
    min_dist = std::min(min_dist, pointSegmentDistance(center, a, b));
  }

  return min_dist - (obstacle_radius + body_radius);
}

double urecaComputeMinInterRobotClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                               double body_radius)
{
  if (trajectory.empty())
    return std::numeric_limits<double>::quiet_NaN();

  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto& q : trajectory)
  {
    Eigen::Vector3d r1(q(0), q(1), q(2));
    Eigen::Vector3d r2(q(3), q(4), q(5));
    min_clearance = std::min(min_clearance, (r1 - r2).norm() - (2.0 * body_radius));
  }

  return min_clearance;
}

std::vector<double> urecaComputePerStepObstacleClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                              const Eigen::Vector3d& obstacle_center,
                                                              double obstacle_radius,
                                                              double body_radius,
                                                              int offset,
                                                              int dim)
{
  std::vector<double> clearances;
  clearances.reserve(trajectory.size());

  for (const auto& q : trajectory)
  {
    double distance = 0.0;
    if (dim == 2)
    {
      const Eigen::Vector2d point(q(offset + 0), q(offset + 1));
      const Eigen::Vector2d center(obstacle_center.x(), obstacle_center.y());
      distance = (point - center).norm();
    }
    else
    {
      const Eigen::Vector3d point(q(offset + 0), q(offset + 1), q(offset + 2));
      distance = (point - obstacle_center).norm();
    }

    clearances.push_back(distance - (obstacle_radius + body_radius));
  }

  return clearances;
}

std::vector<double> urecaComputePerStepInterRobotClearanceProxy(const std::vector<Eigen::VectorXd>& trajectory,
                                                                double body_radius)
{
  std::vector<double> clearances;
  clearances.reserve(trajectory.size());

  for (const auto& q : trajectory)
  {
    const Eigen::Vector3d r1(q(0), q(1), q(2));
    const Eigen::Vector3d r2(q(3), q(4), q(5));
    clearances.push_back((r1 - r2).norm() - (2.0 * body_radius));
  }

  return clearances;
}

void urecaEnsureResultLayout(const std::filesystem::path& result_dir)
{
  std::filesystem::create_directories(result_dir / "trajectories");
  std::filesystem::create_directories(result_dir / "figures");
}

void urecaWriteSummaryCsv(const std::filesystem::path& csv_path, const std::vector<UrecaSummaryRow>& rows)
{
  std::ofstream output(csv_path);
  output << "case_id,K,k1,k2,perturbation,penetration_label,"
            "trajopt_success,trajopt_message,solve_time_ms,collision_free,"
            "post_path_length,post_smoothness_proxy,post_min_clearance_proxy,"
            "post_min_obstacle_clearance_proxy,post_min_inter_robot_clearance_proxy,"
            "trajectory_file\n";

  for (const auto& row : rows)
  {
    output << escapeCsv(row.case_id) << ","
           << row.K << ","
           << row.k1 << ","
           << row.k2 << ","
           << formatDouble(row.perturbation) << ","
           << escapeCsv(row.penetration_label) << ","
           << (row.trajopt_success ? 1 : 0) << ","
           << escapeCsv(row.trajopt_message) << ","
           << formatDouble(row.solve_time_ms) << ","
           << (row.collision_free ? 1 : 0) << ","
           << formatDouble(row.post_path_length) << ","
           << formatDouble(row.post_smoothness_proxy) << ","
           << formatDouble(row.post_min_clearance_proxy) << ","
           << formatDouble(row.post_min_obstacle_clearance_proxy) << ","
           << formatDouble(row.post_min_inter_robot_clearance_proxy) << ","
           << escapeCsv(row.trajectory_file) << "\n";
  }
}

void urecaWriteSingleRobotTrajectoryCsv(const std::filesystem::path& csv_path,
                                        const std::vector<Eigen::VectorXd>& trajectory,
                                        const std::vector<double>& clearance_proxy,
                                        int dim)
{
  std::ofstream output(csv_path);
  output << "step,x,y,z,clearance_proxy\n";

  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    const double x = trajectory[i](0);
    const double y = trajectory[i](1);
    const double z = (dim > 2) ? trajectory[i](2) : 0.0;
    output << i << "," << formatDouble(x) << "," << formatDouble(y) << ","
           << formatDouble(z) << "," << formatDouble(clearance_proxy[i]) << "\n";
  }
}

void urecaWriteTwoRobotTrajectoryCsv(const std::filesystem::path& csv_path,
                                     const std::vector<Eigen::VectorXd>& trajectory,
                                     const std::vector<double>& obstacle_clearance_proxy,
                                     const std::vector<double>& inter_robot_clearance_proxy)
{
  std::ofstream output(csv_path);
  output << "step,r1_x,r1_y,r1_z,r2_x,r2_y,r2_z,obstacle_clearance_proxy,inter_robot_clearance_proxy\n";

  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    output << i << ","
           << formatDouble(trajectory[i](0)) << ","
           << formatDouble(trajectory[i](1)) << ","
           << formatDouble(trajectory[i](2)) << ","
           << formatDouble(trajectory[i](3)) << ","
           << formatDouble(trajectory[i](4)) << ","
           << formatDouble(trajectory[i](5)) << ","
           << formatDouble(obstacle_clearance_proxy[i]) << ","
           << formatDouble(inter_robot_clearance_proxy[i]) << "\n";
  }
}

}  // namespace tesseract_examples
