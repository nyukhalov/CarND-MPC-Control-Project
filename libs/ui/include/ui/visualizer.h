#pragma once

#include "window.h"

#include <mpc/config.h>
#include <mpc/models.h>

#include <vector>

namespace carnd
{

class Visualizer
{
public:
  Visualizer() = delete;
  Visualizer(const Window& window, const MPCConfig& mpc_config);

  void visualize(const MpcSolution& solution,
                 const std::vector<double> ref_pts_x,
                 const std::vector<double> ref_pts_y,
                 const std::vector<double> ref_fitted_pts_x,
                 const std::vector<double> ref_fitted_pts_y) const;
private:
  const MPCConfig& _mpc_config;
  const Window& _window;
};

} // namespace carnd
