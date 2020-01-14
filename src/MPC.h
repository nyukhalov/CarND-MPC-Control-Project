#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "models.h"

using namespace carnd;

class MPC {
public:
  MPC(double target_vel);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  MpcSolution solve(const Eigen::VectorXd &state,
                    const Eigen::VectorXd &coeffs);

private:
  const double _target_vel;
};

#endif  // MPC_H
