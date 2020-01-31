#pragma once

#include "MPCConfig.h"
#include "models.h"
#include "../Eigen-3.3/Eigen/Core"

#include <cppad/cppad.hpp>

using CppAD::AD;
using Eigen::VectorXd;

namespace carnd
{

class FG_eval {
public:

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // Fitted polynomial coefficients
  const VectorXd coeffs;
  const MPCConfig& config;

  FG_eval() = delete;
  FG_eval(VectorXd coeffs_in, const MPCConfig& config_in)
  : coeffs(coeffs_in),
    config(config_in)
  {}

  // `fg` is a vector of the cost constraints
  // `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars);
};

} // namespace carnd
