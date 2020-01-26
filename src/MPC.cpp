#include "MPC.h"
#include "helpers.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

std::ostream& operator<<(std::ostream& os, const MPCConfig& config) {
  os << "MPCConfig{"
     << " num_states=" << config.num_states
     << " num_actuations=" << config.num_actuations
     << " dt=" << config.dt
     << " target_vel=" << config.target_vel
     << " lf=" << config.lf
     << "}";
  return os;
}


class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  const MPCConfig& config;

  FG_eval(VectorXd coeffs, const MPCConfig& _config): config(_config) {
    this->coeffs = coeffs;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` is a vector of the cost constraints
  // `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {
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
};

//
// MPC class definition implementation.
//
MPC::MPC(const MPCConfig config): _config(config) {}
MPC::~MPC() {}

MpcSolution MPC::solve(const VectorXd &state, const VectorXd &coeffs) const {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t num_state_vars = 4;
  size_t num_actuation_vars = 2;
  size_t n_vars = (_config.num_states * num_state_vars) + (_config.num_actuations * num_actuation_vars);
  // Number of constraints
  size_t n_constraints = _config.num_states * num_state_vars;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[_config.x_start] = x;
  vars[_config.y_start] = y;
  vars[_config.psi_start] = psi;
  vars[_config.v_start] = v;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < _config.delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (size_t i = 0; i < _config.num_actuations; ++i) {
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    vars_lowerbound[_config.delta_start + i] = -0.436332;
    vars_upperbound[_config.delta_start + i] = 0.436332;

    // Acceleration/decceleration upper and lower limits.
    vars_lowerbound[_config.a_start + i] = -1.0;
    vars_upperbound[_config.a_start + i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[_config.x_start] = x;
  constraints_lowerbound[_config.y_start] = y;
  constraints_lowerbound[_config.psi_start] = psi;
  constraints_lowerbound[_config.v_start] = v;

  constraints_upperbound[_config.x_start] = x;
  constraints_upperbound[_config.y_start] = y;
  constraints_upperbound[_config.psi_start] = psi;
  constraints_upperbound[_config.v_start] = v;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, _config);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  std::vector<Pose> traj;
  for (size_t i=0; i<_config.num_states; i++) {
    double x = solution.x[_config.x_start + i];
    double y = solution.x[_config.y_start + i];
    double heading = solution.x[_config.psi_start + i];
    traj.push_back({x, y, heading});
  }
  double steering = solution.x[_config.delta_start];
  double throttle = solution.x[_config.a_start];
  return {ok, steering, throttle, traj};
}
