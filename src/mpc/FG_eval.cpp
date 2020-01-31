#include "FG_eval.h"
#include "MPCConfig.h"
#include "../helpers.h"

using namespace carnd;

// `fg` is a vector of the cost constraints
// `vars` is a vector of variable values (state & actuators)
void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
  // The cost is stored is the first element of `fg`.
  // Any additions to the cost should be added to `fg[0]`.
  fg[0] = 0;

  // Reference State Cost
  for (size_t t = 0; t < config.num_states; ++t) {
    AD<double> x = vars[t + config.x_start];
    AD<double> y = vars[t + config.y_start];
    AD<double> y_des = polyeval(coeffs, x);
    AD<double> cte = y_des - y;

    AD<double> psi = vars[t + config.psi_start];
    AD<double> psi_des = CppAD::atan(polyderiveval(coeffs, x));
    AD<double> epsi = psi - psi_des;

    AD<double> vel_err = vars[t + config.v_start] - config.target_vel;

    fg[0] += CppAD::pow(cte, 2);
    fg[0] += CppAD::pow(epsi, 2);
    fg[0] += CppAD::pow(vel_err, 2);
  }

  for (size_t t = 0; t < config.num_actuations; ++t) {
    fg[0] += 100 * CppAD::pow(vars[t + config.delta_start], 2);
    fg[0] += CppAD::pow(vars[t + config.a_start], 2);
  }

  for (size_t t = 1; t < config.num_actuations; ++t) {
    fg[0] += 100 * CppAD::pow(vars[t + config.delta_start] - vars[t - 1 + config.delta_start], 2);
    fg[0] += CppAD::pow(vars[t + config.a_start] - vars[t - 1 + config.a_start], 2);
  }

  //
  // Setup Constraints

  // Initial constraints
  //
  // We add 1 to each of the starting indices due to cost being located at
  // index 0 of `fg`.
  // This bumps up the position of all the other values.
  fg[1 + config.x_start] = vars[config.x_start];
  fg[1 + config.y_start] = vars[config.y_start];
  fg[1 + config.psi_start] = vars[config.psi_start];
  fg[1 + config.v_start] = vars[config.v_start];

  // The rest of the constraints
  for (size_t t = 1; t < config.num_states; ++t) {
    AD<double> x1     = vars[t + config.x_start];
    AD<double> y1     = vars[t + config.y_start];
    AD<double> psi1   = vars[t + config.psi_start];
    AD<double> v1     = vars[t + config.v_start];

    AD<double> x0         = vars[t - 1 + config.x_start];
    AD<double> y0         = vars[t - 1 + config.y_start];
    AD<double> psi0       = vars[t - 1 + config.psi_start];
    AD<double> v0         = vars[t - 1 + config.v_start];
    AD<double> delta0     = vars[t - 1 + config.delta_start];
    AD<double> a0         = vars[t - 1 + config.a_start];
    AD<double> y0_desired   = polyeval(coeffs, x0);

    fg[1 + t + config.x_start] = x1 - (x0 + v0 * CppAD::cos(psi0) * config.dt);
    fg[1 + t + config.y_start] = y1 - (y0 + v0 * CppAD::sin(psi0) * config.dt);
    fg[1 + t + config.psi_start] = psi1 - (psi0 + CppAD::tan(delta0) * (v0/config.lf) * config.dt);
    fg[1 + t + config.v_start] = v1 - (v0 + a0 * config.dt);
  }
}
