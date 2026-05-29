#ifndef TESSERACT_EXAMPLES_TWO_D_OBSTACLE_EXAMPLE_H
#define TESSERACT_EXAMPLES_TWO_D_OBSTACLE_EXAMPLE_H

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief Minimal 2D TrajOpt obstacle-avoidance sanity-check example.
 *
 * The robot is a 2-DOF planar point/sphere robot controlled by joint_x and joint_y.
 * A fixed spherical obstacle is placed near the straight-line path from start to goal.
 *
 * This example does NOT enforce a subgoal. It only fixes start and goal, seeds the
 * intermediate trajectory so that it slightly intersects the obstacle, and checks
 * whether TrajOpt moves the intermediate waypoints around the obstacle.
 */
class TwoDObstacleExample : public Example
{
public:
  TwoDObstacleExample(std::shared_ptr<tesseract_environment::Environment> env,
                      std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr,
                      bool ifopt = false,
                      bool debug = false);

  bool run() override final;

private:
  bool ifopt_{ false };
  bool debug_{ false };
};
}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_TWO_D_OBSTACLE_EXAMPLE_H
