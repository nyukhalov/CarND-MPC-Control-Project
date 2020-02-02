#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace carnd
{

class Window {
public:
  Window(size_t width, size_t height, std::string name);

  void circle(cv::Point center, size_t radius, cv::Scalar color, int thickness=1) const;
  void line(cv::Point from, cv::Point to, cv::Scalar color) const;
  void text(const std::string& text, cv::Point at, cv::Scalar color) const;
  void draw() const;

  const size_t width;
  const size_t height;

private:
  const std::string _name;
  const cv::Scalar _background_color;
  mutable cv::Mat _frame;

  void reset_frame() const;
};

} // namespace carnd
