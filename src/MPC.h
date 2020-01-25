#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "models.h"

using namespace carnd;

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

  MPCConfig(size_t num_states, double dt, double target_vel, double lf):
    num_states(num_states),
    num_actuations(num_states - 1),
    dt(dt),
    target_vel(target_vel),
    lf(lf),
    x_start(0),
    y_start(x_start + num_states),
    psi_start(y_start + num_states),
    v_start(psi_start + num_states),
    delta_start(v_start + num_states),
    a_start(delta_start + num_actuations)
   {}
};

class MPC {
public:
  MPC(const MPCConfig& config);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  MpcSolution solve(const Eigen::VectorXd &state,
                    const Eigen::VectorXd &coeffs) const;

private:
  const MPCConfig& _config;
};

#endif  // MPC_H
