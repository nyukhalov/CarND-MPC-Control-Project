#pragma once

#include <vector>
#include <iostream>

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

std::ostream& operator<<(std::ostream& os, const Pose& pose);
std::ostream& operator<<(std::ostream& os, const MpcSolution& solution);

} // namespace carnd
