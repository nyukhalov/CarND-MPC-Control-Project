#include "window.h"

#include <opencv2/opencv.hpp>

using namespace carnd;

Window::Window(size_t width_in, size_t height_in, std::string name_in):
    width(width_in),
    height(height_in),
    _name(name_in),
    _background_color(cv::Scalar(0, 0, 0))
  {
    cv::namedWindow(_name, cv::WINDOW_AUTOSIZE);
    reset_frame();
  }

void Window::circle(cv::Point center, size_t radius, cv::Scalar color, int thickness) const {
  cv::circle(_frame, center, radius, color, thickness);
}

void Window::line(cv::Point from, cv::Point to, cv::Scalar color) const {
  cv::line(_frame, from, to, color);
}

void Window::text(const std::string& text, cv::Point at, cv::Scalar color) const {
  cv::putText(_frame, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.7, color);
}

void Window::draw() const {
  imshow(_name, _frame);
  cv::waitKey(1);
  reset_frame();
}

void Window::await() const {
  cv::waitKey(-1);
}

void Window::reset_frame() const {
  _frame = cv::Mat(height, width, CV_8UC3, _background_color);
}
