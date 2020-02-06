#pragma once

#include <string>
#include "Eigen/Dense"

using Eigen::VectorXd;
using std::string;

double deg2rad(double x);

// Evaluate a polynomial.
template <class Type>
Type polyeval(const VectorXd &coeffs, const Type& x) {
  Type result = 0.0;
  Type x_i = 1.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * x_i;
    x_i *= x;
  }
  return result;
}

// Evaluate a polynomial first derivative.
template <class Type>
Type polyderiveval(const VectorXd &coeffs, const Type& x) {
  Type result = 0.0;
  Type x_i = 1.0;
  for (int i = 1; i < coeffs.size(); ++i) {
    result += i * coeffs[i] * x_i;
    x_i *= x;
  }
  return result;
}

// Fit a polynomial.
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);
