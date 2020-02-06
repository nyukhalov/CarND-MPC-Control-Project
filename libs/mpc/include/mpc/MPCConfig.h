#pragma once

#include <iostream>

namespace carnd
{

struct MPCConfig {
  const size_t num_states;
  const size_t num_actuations;
  const double dt;
  const double target_vel;

  // This is the length from front to CoG that has a similar radius.
  const double lf;

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  const size_t x_start;
  const size_t y_start;
  const size_t psi_start;
  const size_t v_start;
  const size_t delta_start;
  const size_t a_start;

  MPCConfig() = delete;
  MPCConfig(size_t num_states_in, double dt_in, double target_vel_in, double lf_in):
    num_states(num_states_in),
    num_actuations(num_states_in - 1),
    dt(dt_in),
    target_vel(target_vel_in),
    lf(lf_in),
    x_start(0),
    y_start(x_start + num_states),
    psi_start(y_start + num_states),
    v_start(psi_start + num_states),
    delta_start(v_start + num_states),
    a_start(delta_start + num_actuations)
   {}
};

std::ostream& operator<<(std::ostream& os, const MPCConfig& config);

} // namespace carnd
