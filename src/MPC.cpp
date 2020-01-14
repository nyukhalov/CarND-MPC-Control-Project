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

size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const unsigned int num_states = N;
const unsigned int num_actuations = N - 1;
// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + num_states;
size_t psi_start = y_start + num_states;
size_t v_start = psi_start + num_states;
size_t cte_start = v_start + num_states;
size_t epsi_start = cte_start + num_states;
size_t delta_start = epsi_start + num_states;
size_t a_start = delta_start + num_actuations;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  const double _target_vel;

  FG_eval(VectorXd coeffs, double target_vel): _target_vel(target_vel) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` is a vector of the cost constraints
  // `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    for (int t = 0; t < N; ++t) {
      AD<double> cte = vars[t + cte_start];
      AD<double> epsi = vars[t + epsi_start];
      AD<double> vel_err = vars[t + v_start] - _target_vel;
      fg[0] += CppAD::pow(cte, 2);
      fg[0] += CppAD::pow(epsi, 2);
      fg[0] += CppAD::pow(vel_err, 2);
    }

    for (int t = 0; t < N - 1; ++t) {
      fg[0] += 100 * CppAD::pow(vars[t + delta_start], 2);
      fg[0] += CppAD::pow(vars[t + a_start], 2);
    }

    // for (int t = 1; t < N - 1; ++t) {
    //   fg[0] += 100 * CppAD::pow(vars[t + delta_start] - vars[t - 1 + delta_start], 2);
    //   fg[0] += CppAD::pow(vars[t + a_start] - vars[t - 1 + a_start], 2);
    // }


    //
    // Setup Constraints

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; ++t) {
      AD<double> x1     = vars[t + x_start];
      AD<double> y1     = vars[t + y_start];
      AD<double> psi1   = vars[t + psi_start];
      AD<double> v1     = vars[t + v_start];
      AD<double> cte1   = vars[t + cte_start];
      AD<double> epsi1  = vars[t + epsi_start];

      AD<double> x0         = vars[t - 1 + x_start];
      AD<double> y0         = vars[t - 1 + y_start];
      AD<double> psi0       = vars[t - 1 + psi_start];
      AD<double> v0         = vars[t - 1 + v_start];
      AD<double> delta0     = vars[t - 1 + delta_start];
      AD<double> a0         = vars[t - 1 + a_start];
      AD<double> epsi0      = vars[t - 1 + epsi_start];
      AD<double> y0_desired   = polyeval(coeffs, x0);
      AD<double> psi0_desired = CppAD::atan(polyderiveval(coeffs, x0));

      fg[1 + t + x_start] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + t + y_start] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + t + psi_start] = psi1 - (psi0 + CppAD::tan(delta0) * (v0/Lf) * dt);
      fg[1 + t + v_start] = v1 - (v0 + a0 * dt);
      fg[1 + t + cte_start] = cte1 - (y0_desired - y0);
      fg[1 + t + epsi_start] = epsi1 - (psi0 - psi0_desired);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(double target_vel): _target_vel(target_vel) {}
MPC::~MPC() {}

MpcSolution MPC::solve(const VectorXd &state, const VectorXd &coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = (num_states * 6) + (num_actuations * 2);
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = 0; i < num_actuations; ++i) {
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    vars_lowerbound[delta_start + i] = -0.436332;
    vars_upperbound[delta_start + i] = 0.436332;

    // Acceleration/decceleration upper and lower limits.
    vars_lowerbound[a_start + i] = -1.0;
    vars_upperbound[a_start + i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, _target_vel);

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

  std::cout << "State:"
            << " \tCTE: "  << std::setw(7) << solution.x[cte_start]
            << " \tEpsi: " << std::setw(7) << solution.x[epsi_start]
            << " \tVel: "  << std::setw(7) << solution.x[v_start]
            << std::endl;

  for (int i=1; i<N; i++) {
    std::cout << "\t[A " << std::setw(2) << i << "]"
              << " \tsteering: " << std::setw(7) << solution.x[delta_start + i - 1]
              << " \taccel: "    << std::setw(7) << solution.x[a_start + i - 1] 
              << std::endl;

    std::cout << "State:"
              << " \tCTE: "  << std::setw(7) << solution.x[cte_start + i]
              << " \tEpsi: " << std::setw(7) << solution.x[epsi_start + i]
              << " \tVel: "  << std::setw(7) << solution.x[v_start + i]
              << std::endl;
  }
  // for (int i=0; i<num_actuations; i++) {
  //   std::cout << "throttle[" << std::setw(2) << i << "]: " << solution.x[a_start + i] << std::endl;
  // }

  Trajectory traj;
  for (int i=0; i<N; i++) {
    traj.ptsx.push_back(solution.x[x_start + i]);
    traj.ptsy.push_back(solution.x[y_start + i]);
  }
  double steering = solution.x[delta_start];
  double throttle = solution.x[a_start];
  return {steering, throttle, traj};
}
