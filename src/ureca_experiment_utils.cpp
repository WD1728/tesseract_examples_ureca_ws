#include <tesseract_examples/ureca_experiment_utils.hpp>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_experiment_metrics.hpp>

#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tesseract_examples
{
namespace
{
std::string trim(const std::string& input)
{
  std::size_t first = 0;
  while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first])) != 0)
    ++first;

  std::size_t last = input.size();
  while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1])) != 0)
    --last;

  return input.substr(first, last - first);
}

std::vector<double> parseDoubleListValue(const std::string& value)
{
  std::string body = trim(value);
  if (!body.empty() && body.front() == '[')
    body.erase(body.begin());
  if (!body.empty() && body.back() == ']')
    body.pop_back();

  std::vector<double> parsed;
  std::stringstream ss(body);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    const std::string trimmed = trim(token);
    if (!trimmed.empty())
      parsed.push_back(std::stod(trimmed));
  }
  return parsed;
}

bool parseBoolValue(const std::string& value)
{
  const std::string normalized = trim(value);
  return (normalized == "true" || normalized == "True" || normalized == "1");
}

Eigen::VectorXd toVector(const std::vector<double>& values, int dim)
{
  if (static_cast<int>(values.size()) != dim)
    throw std::runtime_error("Unexpected vector dimension while loading URECA config.");

  Eigen::VectorXd out(dim);
  for (int i = 0; i < dim; ++i)
    out(i) = values[static_cast<std::size_t>(i)];
  return out;
}

Eigen::Vector2d toVector2(const std::vector<double>& values)
{
  const Eigen::VectorXd vec = toVector(values, 2);
  return Eigen::Vector2d(vec(0), vec(1));
}

Eigen::Vector3d toVector3(const std::vector<double>& values)
{
  const Eigen::VectorXd vec = toVector(values, 3);
  return Eigen::Vector3d(vec(0), vec(1), vec(2));
}

std::string replaceOne(const std::string& input, const std::string& needle, const std::string& replacement)
{
  std::size_t pos = input.find(needle);
  if (pos == std::string::npos)
    throw std::runtime_error("Failed to find expected template token while preparing URECA URDF.");

  std::string out = input;
  out.replace(pos, needle.size(), replacement);
  return out;
}

std::string readFile(const std::filesystem::path& path)
{
  std::ifstream input(path);
  if (!input.is_open())
    throw std::runtime_error("Failed to open file: " + path.string());

  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void writeText(const std::filesystem::path& path, const std::string& text)
{
  std::ofstream output(path);
  if (!output.is_open())
    throw std::runtime_error("Failed to write file: " + path.string());

  output << text;
}

std::string formatDouble(double value)
{
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(6);
  ss << value;
  return ss.str();
}

std::string buildTemplatedUrdf(const UrecaSceneConfig& scene)
{
  std::string urdf = readFile(scene.urdf_template);

  const std::string old_origin =
      (scene.dim == 2) ? "<origin xyz=\"1.0 0.6 0.0\" rpy=\"0 0 0\"/>" : "<origin xyz=\"0.5 0.6 0.0\" rpy=\"0 0 0\"/>";
  const std::string new_origin = "<origin xyz=\"" + formatDouble(scene.obstacle_center.x()) + " " +
                                 formatDouble(scene.obstacle_center.y()) + " " + formatDouble(scene.obstacle_center.z()) +
                                 "\" rpy=\"0 0 0\"/>";
  urdf = replaceOne(urdf, old_origin, new_origin);

  const std::size_t obstacle_link_pos = urdf.find("<link name=\"obstacle_link\">");
  if (obstacle_link_pos == std::string::npos)
    throw std::runtime_error("Failed to find obstacle link in URECA URDF template.");

  const std::size_t obstacle_radius_pos = urdf.find("<sphere radius=\"0.20\"/>", obstacle_link_pos);
  if (obstacle_radius_pos == std::string::npos)
    throw std::runtime_error("Failed to find obstacle radius in URECA URDF template.");

  urdf.replace(obstacle_radius_pos,
               std::string("<sphere radius=\"0.20\"/>").size(),
               "<sphere radius=\"" + formatDouble(scene.obstacle_radius) + "\"/>");

  const std::size_t second_radius_pos = urdf.find("<sphere radius=\"0.20\"/>", obstacle_radius_pos + 1);
  if (second_radius_pos != std::string::npos)
  {
    urdf.replace(second_radius_pos,
                 std::string("<sphere radius=\"0.20\"/>").size(),
                 "<sphere radius=\"" + formatDouble(scene.obstacle_radius) + "\"/>");
  }

  return urdf;
}

}  // namespace

std::map<std::string, std::string> parseFlatConfig(const std::filesystem::path& config_path)
{
  std::map<std::string, std::string> out;
  std::ifstream input(config_path);
  if (!input.is_open())
    throw std::runtime_error("Failed to open URECA config: " + config_path.string());

  std::string line;
  while (std::getline(input, line))
  {
    const std::size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos)
      line = line.substr(0, comment_pos);

    line = trim(line);
    if (line.empty())
      continue;

    const std::size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
      continue;

    const std::string key = trim(line.substr(0, colon_pos));
    const std::string value = trim(line.substr(colon_pos + 1));
    out[key] = value;
  }
  return out;
}

UrecaSuiteConfig loadUrecaSuiteConfig(const std::filesystem::path& config_path)
{
  UrecaSuiteConfig config;
  const std::map<std::string, std::string> values = parseFlatConfig(config_path);

  auto assignInt = [&values](const std::string& key, int& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = std::stoi(it->second);
  };
  auto assignDouble = [&values](const std::string& key, double& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = std::stod(it->second);
  };
  auto assignBool = [&values](const std::string& key, bool& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = parseBoolValue(it->second);
  };
  auto assignVector2 = [&values](const std::string& key, Eigen::Vector2d& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = toVector2(parseDoubleListValue(it->second));
  };
  auto assignVector3 = [&values](const std::string& key, Eigen::Vector3d& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = toVector3(parseDoubleListValue(it->second));
  };
  auto assignList = [&values](const std::string& key, std::vector<double>& target) {
    auto it = values.find(key);
    if (it != values.end())
      target = parseDoubleListValue(it->second);
  };

  assignInt("default_n_steps_2d", config.default_n_steps_2d);
  assignInt("default_n_steps_3d", config.default_n_steps_3d);
  assignVector2("two_d_start", config.two_d_start);
  assignVector2("two_d_goal", config.two_d_goal);
  assignVector2("two_d_subgoal", config.two_d_subgoal);
  assignVector2("two_d_obstacle_center", config.two_d_obstacle_center);
  assignDouble("two_d_obstacle_radius", config.two_d_obstacle_radius);
  assignDouble("two_d_robot_radius", config.two_d_robot_radius);
  assignDouble("two_d_velocity_coeff", config.two_d_velocity_coeff);
  assignDouble("two_d_collision_cost_margin", config.two_d_collision_cost_margin);
  assignDouble("two_d_collision_cost_weight", config.two_d_collision_cost_weight);
  assignBool("two_d_collision_constraint_enabled", config.two_d_collision_constraint_enabled);
  assignDouble("two_d_collision_constraint_margin", config.two_d_collision_constraint_margin);
  assignDouble("two_d_collision_constraint_weight", config.two_d_collision_constraint_weight);

  assignVector3("three_d_start", config.three_d_start);
  assignVector3("three_d_goal", config.three_d_goal);
  assignVector3("three_d_subgoal", config.three_d_subgoal);
  assignVector3("three_d_obstacle_center", config.three_d_obstacle_center);
  assignDouble("three_d_obstacle_radius", config.three_d_obstacle_radius);
  assignDouble("three_d_robot_radius", config.three_d_robot_radius);
  assignDouble("three_d_velocity_coeff", config.three_d_velocity_coeff);
  assignDouble("three_d_collision_cost_margin", config.three_d_collision_cost_margin);
  assignDouble("three_d_collision_cost_weight", config.three_d_collision_cost_weight);
  assignBool("three_d_collision_constraint_enabled", config.three_d_collision_constraint_enabled);
  assignDouble("three_d_collision_constraint_margin", config.three_d_collision_constraint_margin);
  assignDouble("three_d_collision_constraint_weight", config.three_d_collision_constraint_weight);

  assignInt("default_selected_k_2d", config.default_selected_k_2d);
  assignInt("default_selected_k_3d", config.default_selected_k_3d);
  assignInt("default_early_k", config.default_early_k);
  assignInt("default_late_k_offset", config.default_late_k_offset);
  assignInt("blackbox_demo_k", config.blackbox_demo_k);

  assignList("penetration_targets", config.penetration_targets);
  assignList("symmetry_perturbations", config.symmetry_perturbations);
  assignList("obstacle_radius_values", config.obstacle_radius_values);
  assignList("safe_margin_values", config.safe_margin_values);
  assignList("two_d_subgoal_y_values", config.two_d_subgoal_y_values);
  assignList("three_d_subgoal_z_values", config.three_d_subgoal_z_values);

  return config;
}

std::vector<std::string> supportedUrecaExperimentIds()
{
  return { "E0_reproduce_current",
           "E1_2d_k_sweep",
           "E2_3d_k_sweep",
           "E3_no_subgoal_vs_subgoal",
           "E4_seed_penetration_depth",
           "E5_symmetry_breaking",
           "E6_obstacle_difficulty_sweep",
           "E7_subgoal_location_sweep",
           "E8_blackbox_wrapper_demo" };
}

UrecaSceneConfig makeBase2DScene(const UrecaSuiteConfig& config, const std::filesystem::path& workspace_root)
{
  UrecaSceneConfig scene;
  scene.scene_id = "2d";
  scene.dim = 2;
  scene.n_steps = config.default_n_steps_2d;
  scene.joint_names = { "joint_x", "joint_y" };
  scene.urdf_template = workspace_root / "support/urdf/ureca_two_d_robot.urdf";
  scene.srdf_template = workspace_root / "support/urdf/ureca_two_d_robot.srdf";
  scene.start = config.two_d_start;
  scene.goal = config.two_d_goal;
  scene.subgoal = Eigen::VectorXd(config.two_d_subgoal);
  scene.obstacle_center = Eigen::Vector3d(config.two_d_obstacle_center.x(), config.two_d_obstacle_center.y(), 0.0);
  scene.obstacle_radius = config.two_d_obstacle_radius;
  scene.robot_radius = config.two_d_robot_radius;
  scene.safe_margin = 0.0;
  scene.collision_cost_margin = config.two_d_collision_cost_margin;
  scene.collision_cost_weight = config.two_d_collision_cost_weight;
  scene.collision_constraint_enabled = config.two_d_collision_constraint_enabled;
  scene.collision_constraint_margin = config.two_d_collision_constraint_margin;
  scene.collision_constraint_weight = config.two_d_collision_constraint_weight;
  scene.velocity_coeff = config.two_d_velocity_coeff;
  return scene;
}

UrecaSceneConfig makeBase3DScene(const UrecaSuiteConfig& config, const std::filesystem::path& workspace_root)
{
  UrecaSceneConfig scene;
  scene.scene_id = "3d";
  scene.dim = 3;
  scene.n_steps = config.default_n_steps_3d;
  scene.joint_names = { "joint_x", "joint_y", "joint_z" };
  scene.urdf_template = workspace_root / "support/urdf/ureca_three_d_robot.urdf";
  scene.srdf_template = workspace_root / "support/urdf/ureca_three_d_robot.srdf";
  scene.start = config.three_d_start;
  scene.goal = config.three_d_goal;
  scene.subgoal = Eigen::VectorXd(config.three_d_subgoal);
  scene.obstacle_center = config.three_d_obstacle_center;
  scene.obstacle_radius = config.three_d_obstacle_radius;
  scene.robot_radius = config.three_d_robot_radius;
  scene.safe_margin = 0.0;
  scene.collision_cost_margin = config.three_d_collision_cost_margin;
  scene.collision_cost_weight = config.three_d_collision_cost_weight;
  scene.collision_constraint_enabled = config.three_d_collision_constraint_enabled;
  scene.collision_constraint_margin = config.three_d_collision_constraint_margin;
  scene.collision_constraint_weight = config.three_d_collision_constraint_weight;
  scene.velocity_coeff = config.three_d_velocity_coeff;
  return scene;
}

int clampSubgoalStep(int requested_k, int n_steps)
{
  return std::max(1, std::min(requested_k, n_steps - 2));
}

std::shared_ptr<tesseract_environment::Environment> createEnvironmentForScene(const UrecaSceneConfig& scene,
                                                                              const std::filesystem::path& runtime_dir,
                                                                              std::string& status_message)
{
  status_message.clear();
  try
  {
    std::filesystem::create_directories(runtime_dir);

    const std::filesystem::path runtime_urdf = runtime_dir / (scene.scene_id + "_runtime.urdf");
    const std::filesystem::path runtime_srdf = runtime_dir / (scene.scene_id + "_runtime.srdf");
    writeText(runtime_urdf, buildTemplatedUrdf(scene));
    writeText(runtime_srdf, readFile(scene.srdf_template));

    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<tesseract_environment::Environment>();
    if (!env->init(runtime_urdf, runtime_srdf, locator))
    {
      status_message = "Failed to initialize environment from generated URDF/SRDF.";
      return nullptr;
    }

    if (!env->setActiveDiscreteContactManager("BulletDiscreteBVHManager"))
    {
      status_message = "Failed to set BulletDiscreteBVHManager.";
      return nullptr;
    }

    return env;
  }
  catch (const std::exception& e)
  {
    status_message = e.what();
    CONSOLE_BRIDGE_logError("%s", status_message.c_str());
    return nullptr;
  }
}

std::vector<Eigen::VectorXd> makeLinearSeed(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, int n_steps)
{
  std::vector<Eigen::VectorXd> seed;
  seed.reserve(static_cast<std::size_t>(n_steps));
  for (int i = 0; i < n_steps; ++i)
  {
    const double alpha = static_cast<double>(i) / static_cast<double>(n_steps - 1);
    seed.push_back(start + alpha * (goal - start));
  }
  return seed;
}

std::vector<Eigen::VectorXd> makeSubgoalSeed(const Eigen::VectorXd& start,
                                             const Eigen::VectorXd& subgoal,
                                             const Eigen::VectorXd& goal,
                                             int n_steps,
                                             int subgoal_step)
{
  const int clamped_k = clampSubgoalStep(subgoal_step, n_steps);
  std::vector<Eigen::VectorXd> seed;
  seed.reserve(static_cast<std::size_t>(n_steps));

  for (int i = 0; i < n_steps; ++i)
  {
    if (i <= clamped_k)
    {
      const double alpha = (clamped_k == 0) ? 0.0 : static_cast<double>(i) / static_cast<double>(clamped_k);
      seed.push_back(start + alpha * (subgoal - start));
    }
    else
    {
      const double alpha =
          static_cast<double>(i - clamped_k) / static_cast<double>((n_steps - 1) - clamped_k);
      seed.push_back(subgoal + alpha * (goal - subgoal));
    }
  }

  return seed;
}

std::vector<Eigen::VectorXd> perturbSeed(const std::vector<Eigen::VectorXd>& seed, int axis, double perturbation)
{
  std::vector<Eigen::VectorXd> out = seed;
  if (seed.size() < 3 || perturbation == 0.0)
    return out;

  for (std::size_t i = 1; i + 1 < out.size(); ++i)
  {
    if (axis >= 0 && axis < out[i].size())
      out[i](axis) += perturbation;
  }
  return out;
}

UrecaSummaryRow makeSummaryRow(const std::string& experiment_id,
                               const std::string& case_id,
                               const UrecaSceneConfig& scene,
                               int k,
                               const std::string& seed_type,
                               double perturbation,
                               double obstacle_offset,
                               const std::optional<Eigen::VectorXd>& subgoal,
                               const TrajOptResult& result,
                               const std::string& trajectory_file)
{
  UrecaSummaryRow row;
  row.experiment_id = experiment_id;
  row.case_id = case_id;
  row.dim = scene.dim;
  row.N = scene.n_steps;
  row.K = k;
  row.seed_type = seed_type;
  row.perturbation = perturbation;
  row.obstacle_radius = scene.obstacle_radius;
  row.safe_radius = scene.obstacle_radius + scene.robot_radius + scene.safe_margin;
  row.obstacle_offset = obstacle_offset;
  row.subgoal_x = subgoal.has_value() && subgoal->size() > 0 ? (*subgoal)(0) : 0.0;
  row.subgoal_y = subgoal.has_value() && subgoal->size() > 1 ? (*subgoal)(1) : 0.0;
  row.subgoal_z = subgoal.has_value() && subgoal->size() > 2 ? (*subgoal)(2) : 0.0;
  row.seed_min_clearance = result.seed_min_clearance;
  row.opt_min_clearance = result.success ? result.opt_min_clearance : result.seed_min_clearance;
  row.max_penetration = computeUrecaMaxPenetration(row.opt_min_clearance);
  row.objective = result.objective;
  row.path_length = computeUrecaPathLength(result.success ? result.optimized_trajectory : result.seed_trajectory);
  row.smoothness = computeUrecaSmoothness(result.success ? result.optimized_trajectory : result.seed_trajectory);
  row.solve_time_ms = result.solve_time_ms;
  row.status = result.status_message;
  row.success = result.success;
  row.trajectory_file = trajectory_file;
  return row;
}

std::filesystem::path ensureExperimentSubdir(const UrecaExperimentContext& context, const std::string& experiment_id)
{
  const std::filesystem::path dir = context.output_dir / experiment_id;
  std::filesystem::create_directories(dir);
  return dir;
}

std::vector<std::string> splitCsvList(const std::string& value)
{
  std::vector<std::string> out;
  std::stringstream ss(value);
  std::string item;
  while (std::getline(ss, item, ','))
  {
    const std::string trimmed = trim(item);
    if (!trimmed.empty())
      out.push_back(trimmed);
  }
  return out;
}

}  // namespace tesseract_examples
