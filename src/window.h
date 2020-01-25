#ifndef WINDOW_H
#define WINDOW_H

#include <string>
#include <opencv2/opencv.hpp>

namespace carnd
{

class Window {
public:
  Window(size_t width, size_t height, std::string name);

  void circle(cv::Point center, size_t radius, cv::Scalar color, int thickness=1);
  void draw();

  const size_t width;
  const size_t height;

private:
  const std::string _name;
  const cv::Scalar _background_color;
  cv::Mat _frame;

  void reset_frame();
};

} // namespace carnd

#endif
