/**
 * @file pick_and_place_unified_example_node.cpp
 * @brief Unified pick and place example node
 */
#include <tesseract_visualization/visualization.h>
#include <array>
#include <tesseract_examples/pick_and_place_example.h>
#include <filesystem>
#include <console_bridge/console.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>

using namespace tesseract_examples;
using namespace tesseract_common;
using namespace tesseract_environment;

int main(int /*argc*/, char** /*argv*/)
{
  auto locator = std::make_shared<GeneralResourceLocator>();
  std::filesystem::path urdf_path =
      locator->locateResource("package://tesseract_support/urdf/pick_and_place_plan.urdf")->getFilePath();
  std::filesystem::path srdf_path =
      locator->locateResource("package://tesseract_support/urdf/pick_and_place_plan.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);
  double box_size = 0.2;
  std::array<double,2> box_pos = {0.15, 0.4};

  CONSOLE_BRIDGE_logInform("Unified pick and place example");

  auto plotter = std::make_shared<tesseract_visualization::Visualization>();
  plotter->waitForConnection();

  PickAndPlaceExample example(env, plotter, /*ifopt*/ false, /*debug*/ false, box_size, box_pos);

  if (!example.run())
  {
    CONSOLE_BRIDGE_logError("Unified PickAndPlaceExample failed");
    exit(1);
  }

  CONSOLE_BRIDGE_logInform("Unified PickAndPlaceExample successful");
}
