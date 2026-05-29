/**
 * @file two_d_obstacle_example_node.cpp
 * @brief Standalone node for the minimal 2D TrajOpt obstacle-avoidance sanity check.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/2d_obstacle_example.h>

#include <filesystem>
#include <memory>

#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>

int main(int /*argc*/, char** /*argv*/)
{
  const std::filesystem::path urdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/2d_obstacle_robot.urdf");
  const std::filesystem::path srdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/2d_obstacle_robot.srdf");

  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto env = std::make_shared<tesseract_environment::Environment>();

  if (!env->init(urdf_path, srdf_path, locator))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment from 2D obstacle URDF/SRDF.");
    return 1;
  }

  tesseract_examples::TwoDObstacleExample example(env, nullptr, false, true);

  if (!example.run())
  {
    CONSOLE_BRIDGE_logError("TwoDObstacleExample failed");
    return 1;
  }

  CONSOLE_BRIDGE_logInform("TwoDObstacleExample successful");
  return 0;
}
