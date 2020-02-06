#include "models.h"

using namespace carnd;

std::ostream& carnd::operator<<(std::ostream& os, const Pose& pose)
{
  os << "Pose{"
      << "x=" << pose.x
      << ", y=" << pose.y
      << ", heading=" << pose.heading
     << "}";
  return os;
}

std::ostream& carnd::operator<<(std::ostream& os, const VehicleState& state)
{
  os << "VehicleState{"
      << "pose=" << state.pose
      << ", velocity=" << state.velocity
     << "}";
  return os;
}

std::ostream& carnd::operator<<(std::ostream& os, const MpcSolution& solution)
{
  os << "MpcSolution{" << std::endl
      << "\tsuccess=" << (solution.success? "true": "false") << std::endl
      << "\tsteering=" << solution.steering << std::endl
      << "\tthrottle=" << solution.throttle << std::endl
      << "\ttrajectory=[" << std::endl;

  for(const auto& pose: solution.trajectory)
  {
    os << "\t\t" << pose << std::endl;
  }

  os << "\t]" << std::endl
     << "}";
  return os;
}
