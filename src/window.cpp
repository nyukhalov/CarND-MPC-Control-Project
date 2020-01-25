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

void Window::circle(cv::Point center, size_t radius, cv::Scalar color, int thickness) {
  cv::circle(_frame, center, radius, color, thickness);
}

void Window::line(cv::Point from, cv::Point to, cv::Scalar color) {
  cv::line(_frame, from, to, color);
}

void Window::draw() {
  imshow(_name, _frame);
  cv::waitKey(1);
  reset_frame();
}

void Window::reset_frame() {
  _frame = cv::Mat(height, width, CV_8UC3, _background_color);
}
