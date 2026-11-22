#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class EpipolarView {
 public:
  EpipolarView(const cv::Mat &first, const cv::Mat &second, const cv::Mat &F,
               const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2)
      : first_size_(first.size()), second_size_(second.size()), points1_(points1), points2_(points2),
        background_(542, 1184, CV_8UC3, cv::Scalar(32, 28, 24)) {
    left_ = place_image(first, 12);
    right_ = place_image(second, 600);
    label(background_, "View 1   /   Source point", {12, 32}, 0.75);
    label(background_, "View 2   /   Epipolar line and observed match", {600, 32}, 0.75);
    // OpenCV normalizes each (a, b, c) so that a*a + b*b = 1.
    cv::computeCorrespondEpilines(points1_, 1, F, lines2_);
    for (const auto &p : points1_) {
      cv::Point dot = to_canvas(p, first_size_, left_);
      cv::circle(background_, dot, 3, cv::Scalar(25, 25, 25), -1, cv::LINE_AA);
      cv::circle(background_, dot, 2, cv::Scalar(200, 210, 210), -1, cv::LINE_AA);
    }
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS")) {
      std::cout << "Writing epipolar_geometry.png\n";
      return cv::imwrite("epipolar_geometry.png", render()) ? 0 : 1;
    }
    cv::namedWindow(title_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(title_, background_.cols, background_.rows);
    cv::setMouseCallback(title_, on_click, this);
    cv::imshow(title_, render());
    while (true) {
      const int key = cv::waitKey(50);
      if (key == 27 || key == 'q' || cv::getWindowProperty(title_, cv::WND_PROP_VISIBLE) == 0) break;
      if (key == ' ' || key == 'n') selected_ = (selected_ + 1) % points1_.size();
      else if (key == 'p') selected_ = (selected_ + points1_.size() - 1) % points1_.size();
      else continue;
      cv::imshow(title_, render());
    }
    cv::destroyAllWindows();
    return 0;
  }

 private:
  const std::string title_ = "Epipolar geometry";
  cv::Size first_size_, second_size_;
  std::vector<cv::Point2f> points1_, points2_;
  std::vector<cv::Vec3f> lines2_;
  cv::Mat background_;
  cv::Rect left_, right_;
  size_t selected_ = 0;

  cv::Rect place_image(const cv::Mat &image, int x) {
    const int width = 572, height = 416;
    const double scale = std::min(double(width) / image.cols, double(height) / image.rows);
    const cv::Size size(std::max(1, cvRound(image.cols * scale)),
                        std::max(1, cvRound(image.rows * scale)));
    cv::Rect area(x + (width - size.width) / 2, 48 + (height - size.height) / 2, size.width, size.height);
    cv::resize(image, background_(area), size, 0, 0, scale < 1 ? cv::INTER_AREA : cv::INTER_LINEAR);
    return area;
  }

  static cv::Point to_canvas(cv::Point2f p, cv::Size original, cv::Rect area) {
    return {area.x + cvRound((p.x + 0.5) * area.width / original.width - 0.5),
            area.y + cvRound((p.y + 0.5) * area.height / original.height - 0.5)};
  }

  static void label(cv::Mat &canvas, const std::string &text, cv::Point pos, double scale) {
    cv::putText(canvas, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale, {230, 235, 238}, 1, cv::LINE_AA);
  }

  static void marker(cv::Mat &canvas, cv::Point p, cv::Scalar color) {
    cv::circle(canvas, p, 10, {25, 25, 25}, 5, cv::LINE_AA);
    cv::circle(canvas, p, 10, color, 3, cv::LINE_AA);
    cv::circle(canvas, p, 2, color, -1, cv::LINE_AA);
  }

  cv::Mat render() const {
    cv::Mat canvas = background_.clone();
    const auto line = lines2_[selected_];
    const float a = line[0], b = line[1], c = line[2];
    const cv::Scalar gold(55, 200, 255), cyan(240, 220, 45);
    const auto observed = points2_[selected_];
    std::string residual = "Epipolar line undefined";
    if (a != 0 || b != 0) {
      // Solve along the more stable axis, then clip in original image coordinates.
      const int xmax = second_size_.width - 1, ymax = second_size_.height - 1;
      cv::Point start, end;
      if (std::abs(a) >= std::abs(b)) {
        start = {cvRound(-c / a), 0};
        end = {cvRound(-(b * ymax + c) / a), ymax};
      } else {
        start = {0, cvRound(-c / b)};
        end = {xmax, cvRound(-(a * xmax + c) / b)};
      }
      if (cv::clipLine(second_size_, start, end)) {
        start = to_canvas(start, second_size_, right_);
        end = to_canvas(end, second_size_, right_);
        cv::line(canvas, start, end, {30, 30, 30}, 4, cv::LINE_AA);
        cv::line(canvas, start, end, gold, 2, cv::LINE_AA);
      }
      const double distance = std::abs(a * observed.x + b * observed.y + c);
      residual = cv::format("Point-to-line distance   %.2f px", distance);
    }
    marker(canvas, to_canvas(points1_[selected_], first_size_, left_), gold);
    marker(canvas, to_canvas(observed, second_size_, right_), cyan);
    label(canvas, cv::format("Match %d / %d", int(selected_ + 1), int(points1_.size())), {20, 494}, 0.67);
    label(canvas, "Click a left dot   |   N / P  next / previous   |   Q  close", {20, 526}, 0.53);
    label(canvas, residual, {600, 494}, 0.67);
    label(canvas, "Gold line: prediction      Cyan ring: observed feature", {600, 526}, 0.53);
    return canvas;
  }

  static void on_click(int event, int x, int y, int, void *context) {
    auto &view = *static_cast<EpipolarView *>(context);
    if (event != cv::EVENT_LBUTTONDOWN || !view.left_.contains(cv::Point(x, y))) return;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < view.points1_.size(); ++i) {
      cv::Point dot = to_canvas(view.points1_[i], view.first_size_, view.left_);
      const double distance = cv::norm(dot - cv::Point(x, y));
      if (distance < best) {
        view.selected_ = i;
        best = distance;
      }
    }
    cv::imshow(view.title_, view.render());
  }
};
