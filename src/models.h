#ifndef MODELS_H
#define MODELS_H

#include <vector>

namespace carnd {

typedef std::vector<double> Coords;

struct Trajectory {
  Coords ptsx;
  Coords ptsy;
};

/**
 * @brief contains the first actuation and the best trajectory
 */
struct MpcSolution {
  // steering is in radians
  double steering;
  double throttle;
  Trajectory trajectory;
};

} // namespace carnd

#endif
