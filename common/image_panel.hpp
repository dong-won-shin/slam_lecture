#pragma once

#include <algorithm>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lecture {

inline cv::Rect draw_image(cv::Mat &canvas, cv::Rect bounds, const cv::Mat &image) {
  const double scale = std::min(double(bounds.width) / image.cols,
                                double(bounds.height) / image.rows);
  const cv::Size size(std::max(1, cvRound(image.cols * scale)),
                      std::max(1, cvRound(image.rows * scale)));
  const cv::Rect area(bounds.x + (bounds.width - size.width) / 2,
                      bounds.y + (bounds.height - size.height) / 2,
                      size.width, size.height);
  cv::Mat destination = canvas(area);
  cv::resize(image, destination, size, 0, 0,
             scale < 1 ? cv::INTER_AREA : cv::INTER_LINEAR);
  return area;
}

inline void draw_panel(cv::Mat &canvas, cv::Rect bounds, const cv::Mat &image,
                       const std::string &title) {
  const int header = 30;
  cv::putText(canvas, title, {bounds.x, bounds.y + 20}, cv::FONT_HERSHEY_SIMPLEX,
              0.55, cv::Scalar(238, 238, 238), 1, cv::LINE_AA);
  draw_image(canvas, {bounds.x, bounds.y + header, bounds.width, bounds.height - header}, image);
}

inline void show_result(const std::string &title, const cv::Mat &canvas) {
  cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
  cv::resizeWindow(title, canvas.cols, canvas.rows);
  cv::imshow(title, canvas);
  cv::waitKey(0);
}

} // namespace lecture
