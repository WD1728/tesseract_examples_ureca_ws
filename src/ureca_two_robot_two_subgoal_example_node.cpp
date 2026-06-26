#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_two_robot_two_subgoal_example.hpp>

#include <filesystem>
#include <memory>

#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>

int main(int /*argc*/, char** /*argv*/)
{
  const std::filesystem::path urdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/ureca_two_robot_two_subgoal.urdf");
  const std::filesystem::path srdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/ureca_two_robot_two_subgoal.srdf");

  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto env = std::make_shared<tesseract_environment::Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment from URECA two-robot two-subgoal URDF/SRDF.");
    return 1;
  }

  tesseract_examples::UrecaTwoRobotTwoSubgoalExample example(env, nullptr, false, true);
  if (!example.run())
  {
    CONSOLE_BRIDGE_logError("UrecaTwoRobotTwoSubgoalExample failed");
    return 1;
  }

  CONSOLE_BRIDGE_logInform("UrecaTwoRobotTwoSubgoalExample successful");
  return 0;
}
