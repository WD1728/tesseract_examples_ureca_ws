#ifndef TESSERACT_EXAMPLES_URECA_TWO_ROBOT_TWO_SUBGOAL_EXAMPLE_HPP
#define TESSERACT_EXAMPLES_URECA_TWO_ROBOT_TWO_SUBGOAL_EXAMPLE_HPP

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
class UrecaTwoRobotTwoSubgoalExample : public Example
{
public:
  UrecaTwoRobotTwoSubgoalExample(std::shared_ptr<tesseract_environment::Environment> env,
                                 std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                                 bool ifopt = false,
                                 bool debug = false);

  bool run() override final;

private:
  bool ifopt_{ false };
  bool debug_{ false };
};
}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_URECA_TWO_ROBOT_TWO_SUBGOAL_EXAMPLE_HPP
