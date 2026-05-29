#ifndef TESSERACT_EXAMPLES_THREE_D_EXAMPLE_H
#define TESSERACT_EXAMPLES_THREE_D_EXAMPLE_H

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief 3D single-robot subgoal-timing TrajOpt toy example.
 *
 * This example is the 3D counterpart of the previous 2D subgoal-timing demo.
 * The robot is a 3-DOF point/sphere robot with joint_x, joint_y, and joint_z.
 * A fixed spherical obstacle is placed on the straight-line path from start to goal.
 *
 * The experiment sweeps the timestep k at which the fixed subgoal q_k is enforced.
 */
class ThreeDExample : public Example
{
public:
  ThreeDExample(std::shared_ptr<tesseract_environment::Environment> env,
                std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                bool ifopt = false,
                bool debug = false);

  bool run() override final;

private:
  bool ifopt_{ false };
  bool debug_{ false };
};
}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_THREE_D_EXAMPLE_H
