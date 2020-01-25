#ifndef MODELS_H
#define MODELS_H

#include <vector>

namespace carnd {

struct Pose {
  double x;
  double y;
  double heading;
};

/**
 * @brief contains the first actuation and the best trajectory
 */
struct MpcSolution {
  // steering is in radians
  double steering;
  double throttle;
  std::vector<Pose> trajectory;
};

} // namespace carnd

#endif
