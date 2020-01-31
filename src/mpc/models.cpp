#include "models.h"

using namespace carnd;

std::ostream& carnd::operator<<(std::ostream& os, const Pose& pose)
{
  os << "Pose{"
      << "x=" << pose.x
      << ", y=" << pose.y
      << ", heading=" << pose.heading
      << ", vel=" << pose.velocity
     << "}";
  return os;
}

std::ostream& carnd::operator<<(std::ostream& os, const MpcSolution& solution)
{
  os << "MpcSolution{"
      << "success=" << (solution.success? "true": "false")
      << ", steering=" << solution.steering
      << ", throttle=" << solution.throttle
      << ", trajectory=[";

  for(const auto& pose: solution.trajectory)
  {
    os << pose << ", ";
  }

  os << "]}";
  return os;
}
