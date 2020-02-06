#include "config.h"

using namespace carnd;

std::ostream& carnd::operator<<(std::ostream& os, const MPCConfig& config) {
  os << "MPCConfig{"
     << " num_states=" << config.num_states
     << " num_actuations=" << config.num_actuations
     << " dt=" << config.dt
     << " target_vel=" << config.target_vel
     << " lf=" << config.lf
     << "}";
  return os;
}
