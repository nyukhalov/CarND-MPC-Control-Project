#include "mpc/MPCConfig.h"
#include "mpc/MPC.h"
#include "mpc/models.h"

#include "Eigen-3.3/Eigen/Core"

#include <iostream>

using namespace carnd;
using Eigen::VectorXd;

int main()
{
  size_t num_states = 10;
  double dt = 0.1;
  double target_vel = 50;
  double lf = 2.67;
  MPCConfig config(num_states, dt, target_vel, lf);
  MPC mpc(config);

  VectorXd coeffs(4);
  coeffs << 1, 2, 3, 4;

  double v_px = 0;
  double v_py = 0;
  double v_psi = 0;
  double v = 10;
  VectorXd state(4);
  state << v_px, v_py, v_psi, v;

  MpcSolution solution = mpc.solve(state, coeffs);

  std::cout << solution << std::endl;

  return 0;
}
