#pragma once

#include <vector>

namespace carnd {

struct Pose {
  double x;
  double y;
  double heading;
  double velocity;
};

/**
 * @brief contains the first actuation and the best trajectory
 */
struct MpcSolution {
  bool success;
  double steering; // in radians
  double throttle;
  std::vector<Pose> trajectory;
};

} // namespace carnd
