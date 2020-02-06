#include "visualizer.h"

#include <opencv2/opencv.hpp>

#include <string>

using namespace carnd;

namespace detail
{

class PointTransformer
{
public:
  PointTransformer(double scale_x, double scale_y, size_t window_height, cv::Point origin) : _scale_x(scale_x),
                                                                                             _scale_y(scale_y),
                                                                                             _window_height(window_height),
                                                                                             _origin(origin)
  {
  }

  cv::Point transform(double x, double y) const
  {
    double loc_x = _scale_x * x;
    double loc_y = _scale_y * y;
    double px = _origin.x - loc_y;
    double py = _window_height - _origin.y - loc_x;
    return cv::Point(px, py);
  }

private:
  const double _scale_x;
  const double _scale_y;
  const size_t _window_height;
  const cv::Point _origin;
};

} // namespace detail

Visualizer::Visualizer(const Window &window, const MPCConfig &mpc_config) : _mpc_config(mpc_config),
                                                                            _window(window)
{
}

void Visualizer::visualize(const MpcSolution &solution,
                           const std::vector<double> ref_pts_x,
                           const std::vector<double> ref_pts_y,
                           const std::vector<double> ref_fitted_pts_x,
                           const std::vector<double> ref_fitted_pts_y) const
{
  double scale_x = 8;
  double scale_y = scale_x;
  double x_offset = 0.5 * _window.width;
  double y_offset = 0.1 * _window.height;
  cv::Point origin = cv::Point(x_offset, y_offset);
  detail::PointTransformer pt(scale_x, scale_y, _window.height, origin);

  // drawing the current trajectories
  double ref_fitted_node_radius = 2;
  double ref_node_radius = 4;
  double mpc_node_radius = 8;

  // the colors are in BGR
  cv::Scalar ref_fitted_node_color = cv::Scalar(200, 255, 255);
  cv::Scalar ref_node_color = cv::Scalar(0, 0, 255);
  cv::Scalar heading_color = cv::Scalar(0, 0, 255);

  for (int i = 0; i < ref_fitted_pts_x.size(); i++)
  {
    cv::Point point = pt.transform(ref_fitted_pts_x[i], ref_fitted_pts_y[i]);
    _window.circle(point, ref_fitted_node_radius, ref_fitted_node_color);
  }

  for (int i = 0; i < ref_pts_x.size(); i++)
  {
    cv::Point point = pt.transform(ref_pts_x[i], ref_pts_y[i]);
    _window.circle(point, ref_node_radius, ref_node_color);
  }

  for (const auto &state : solution.trajectory)
  {
    double vel_progress = 1.0 - (_mpc_config.target_vel - state.velocity) / _mpc_config.target_vel;
    cv::Scalar mpc_node_color = cv::Scalar(0, vel_progress * 255, (1 - vel_progress) * 255);
    double heading = M_PI + M_PI / 2 - state.pose.heading;
    cv::Point point = pt.transform(state.pose.x, state.pose.y);
    cv::Point heading_point(
        point.x + mpc_node_radius * cos(heading),
        point.y + mpc_node_radius * sin(heading));
    _window.circle(point, mpc_node_radius, mpc_node_color, -1);
    _window.line(point, heading_point, heading_color);
  }

  std::string status = solution.success ? "OK" : "FAIL";
  cv::Scalar status_color = solution.success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
  cv::Point status_point(30, 50);
  _window.text(status, status_point, status_color);

  _window.draw();
}
