#pragma once

#include "models.h"
#include "MPCConfig.h"

#include "../Eigen-3.3/Eigen/Core"

#include <vector>

namespace carnd
{

class MPC {
public:
  MPC(const MPCConfig config_in);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  MpcSolution solve(const Eigen::VectorXd &state,
                    const Eigen::VectorXd &coeffs) const;

  const MPCConfig config;
};

} // namespace carnd
