#include "helpers.h"
#include "mpc/MPCConfig.h"
#include "mpc/MPC.h"
#include "mpc/models.h"
#include "ui/window.h"
#include "ui/visualizer.h"

#include "Eigen-3.3/Eigen/Core"

#include <iostream>

using namespace carnd;
using Eigen::VectorXd;

int main()
{
  size_t num_states = 10;
  double dt = 0.1;
  double target_vel = 50; double lf = 2.67;
  MPCConfig config(num_states, dt, target_vel, lf);
  MPC mpc(config);

  std::vector<double> ref_pts_x{-1.87535, 3.0215, 9.6864, 16.1163, 21.6955, 26.1257};
  std::vector<double> ref_pts_y{0.477098, 0.00847148, -0.727117, -3.82538, -9.45462, -18.3475};
  VectorXd ref_pts_x_vec = VectorXd::Map(ref_pts_x.data(), ref_pts_x.size());
  VectorXd ref_pts_y_vec = VectorXd::Map(ref_pts_y.data(), ref_pts_y.size());
  VectorXd coeffs = polyfit(ref_pts_x_vec, ref_pts_y_vec, 3);

  std::vector<double> ref_fitted_pts_x;
  std::vector<double> ref_fitted_pts_y;
  for (double x = 0; x < 100; x += 4)
  {
    ref_fitted_pts_x.push_back(x);
    ref_fitted_pts_y.push_back(polyeval(coeffs, x));
  }

  double v_px = 0;
  double v_py = 0;
  double v_psi = 0;
  double v = 49.13823;
  VectorXd state(4);
  state << v_px, v_py, v_psi, v;

  MpcSolution solution = mpc.solve(state, coeffs, ref_pts_x_vec, ref_pts_y_vec);

  std::cout << solution << std::endl;

  Window window(500, 500, "Main");
  Visualizer visualizer(window, mpc.config);

  visualizer.visualize(solution, ref_pts_x, ref_pts_y, ref_fitted_pts_x, ref_fitted_pts_y);

  window.await();

  return 0;
}
