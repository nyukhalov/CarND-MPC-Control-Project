#include "MPC.h"
#include "FG_eval.h"
#include "../helpers.h"

#include "../Eigen-3.3/Eigen/Core"

#include <iostream>
#include <string>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
using Eigen::VectorXd;
using namespace carnd;

//
// MPC class definition implementation.
//
MPC::MPC(const MPCConfig config_in): config(config_in) {}
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
  size_t n_vars = (config.num_states * num_state_vars) + (config.num_actuations * num_actuation_vars);
  // Number of constraints
  size_t n_constraints = config.num_states * num_state_vars;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[config.x_start] = x;
  vars[config.y_start] = y;
  vars[config.psi_start] = psi;
  vars[config.v_start] = v;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < config.delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (size_t i = 0; i < config.num_actuations; ++i) {
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    vars_lowerbound[config.delta_start + i] = -0.436332;
    vars_upperbound[config.delta_start + i] = 0.436332;

    // Acceleration/decceleration upper and lower limits.
    vars_lowerbound[config.a_start + i] = -1.0;
    vars_upperbound[config.a_start + i] = 1.0;
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
  constraints_lowerbound[config.x_start] = x;
  constraints_lowerbound[config.y_start] = y;
  constraints_lowerbound[config.psi_start] = psi;
  constraints_lowerbound[config.v_start] = v;

  constraints_upperbound[config.x_start] = x;
  constraints_upperbound[config.y_start] = y;
  constraints_upperbound[config.psi_start] = psi;
  constraints_upperbound[config.v_start] = v;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, config);

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
  for (size_t i=0; i<config.num_states; i++) {
    double x = solution.x[config.x_start + i];
    double y = solution.x[config.y_start + i];
    double heading = solution.x[config.psi_start + i];
    double velocity = solution.x[config.v_start + i];
    traj.push_back({x, y, heading, velocity});
  }
  double steering = solution.x[config.delta_start];
  double throttle = solution.x[config.a_start];
  return {ok, steering, throttle, traj};
}
