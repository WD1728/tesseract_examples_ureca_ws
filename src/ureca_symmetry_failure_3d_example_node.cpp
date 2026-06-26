#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/ureca_symmetry_failure_3d_example.hpp>

#include <filesystem>
#include <memory>

#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>

int main(int /*argc*/, char** /*argv*/)
{
  const std::filesystem::path urdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/ureca_symmetry_failure_3d.urdf");
  const std::filesystem::path srdf_path(
      "/home/wenda/tesseract_ws/src/tesseract_planning/tesseract_examples/support/urdf/ureca_symmetry_failure_3d.srdf");

  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto env = std::make_shared<tesseract_environment::Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment from URECA symmetry failure URDF/SRDF.");
    return 1;
  }

  tesseract_examples::UrecaSymmetryFailure3DExample example(env, nullptr, false, true);
  if (!example.run())
  {
    CONSOLE_BRIDGE_logError("UrecaSymmetryFailure3DExample failed");
    return 1;
  }

  CONSOLE_BRIDGE_logInform("UrecaSymmetryFailure3DExample successful");
  return 0;
}
