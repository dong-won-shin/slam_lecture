#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lecture {

class OrbitView {
 protected:
  OrbitView(const std::string &title, size_t count) : title_(title), count_(count) {}
  virtual ~OrbitView() = default;
  virtual cv::Mat render() const = 0;
  virtual bool handle_key(int) { return false; }

  const cv::Rect scene_{12, 48, 762, 530};
  size_t selected_ = 0;

  void fit(const std::vector<cv::Vec3d> &positions) {
    cv::Vec3d low = positions.front(), high = low;
    for (const auto &p : positions) {
      for (int k = 0; k < 3; ++k) {
        low[k] = std::min(low[k], p[k]);
        high[k] = std::max(high[k], p[k]);
      }
    }
    pivot_ = (low + high) * 0.5;
    // Keep one scale for all three axes, including the camera baseline.
    double dx = 1.0, dy = 1.0;
    auto measure = [&](cv::Vec3d p) {
      const cv::Point2d q = rotate(p);
      dx = std::max(dx, std::abs(q.x));
      dy = std::max(dy, std::abs(q.y));
    };
    for (const auto &p : positions) measure(p);
    scale_ = std::min((scene_.width / 2 - 55) / dx, (scene_.height / 2 - 42) / dy);
  }

  int show_window() {
    cv::namedWindow(title_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(title_, 1160, 652);
    cv::setMouseCallback(title_, orbit, this);
    cv::imshow(title_, render());
    while (true) {
      const int key = cv::waitKey(40);
      if (key == 27 || key == 'q' || cv::getWindowProperty(title_, cv::WND_PROP_VISIBLE) == 0) break;
      if (handle_key(key)) {
        cv::imshow(title_, render());
        continue;
      }
      switch (key) {
        case ' ': case 'n': selected_ = (selected_ + 1) % count_; break;
        case 'p': selected_ = (selected_ + count_ - 1) % count_; break;
        case '+': case '=': zoom_ = std::min(8.0, zoom_ * 1.25); break;
        case '-': zoom_ = std::max(0.25, zoom_ / 1.25); break;
        case 'r': yaw_ = initial_yaw; pitch_ = initial_pitch; zoom_ = 1.0; break;
        default: continue;
      }
      cv::imshow(title_, render());
    }
    cv::destroyAllWindows();
    return 0;
  }

  cv::Point2d rotate(cv::Vec3d p) const {
    p -= pivot_;
    const double z = -std::sin(yaw_) * p[0] + std::cos(yaw_) * p[2];
    return {std::cos(yaw_) * p[0] + std::sin(yaw_) * p[2],
            std::cos(pitch_) * p[1] - std::sin(pitch_) * z};
  }

  cv::Point project(const cv::Vec3d &p) const {
    const cv::Point2d q = rotate(p) * (scale_ * zoom_);
    return {cvRound(scene_.width / 2 + q.x), cvRound(scene_.height / 2 + q.y)};
  }

  void draw_camera(cv::Mat &scene, const cv::Vec3d &center, const cv::Matx33d &orientation,
              cv::Size size, const cv::Matx33d &invK, double length, cv::Scalar color) const {
    const double w = size.width - 1, h = size.height - 1;
    const cv::Point2d corners[] = {{0, 0}, {w, 0}, {w, h}, {0, h}};
    std::vector<cv::Point> rim;
    for (const auto &pixel : corners) {
      rim.push_back(project(center + orientation * (invK * cv::Vec3d(pixel.x, pixel.y, 1)) * length));
      cv::line(scene, project(center), rim.back(), color, 1, cv::LINE_AA);
    }
    cv::polylines(scene, rim, true, color, 1, cv::LINE_AA);
    cv::circle(scene, project(center), 4, color, -1, cv::LINE_AA);
  }

  static void text(cv::Mat &canvas, const std::string &s, cv::Point p, double size = 0.55) {
    cv::putText(canvas, s, p, cv::FONT_HERSHEY_SIMPLEX, size, {230, 235, 238}, 1, cv::LINE_AA);
  }

  void draw_axes(cv::Mat &scene) const {
    const cv::Point axes_origin(62, 58);
    const cv::Point2d screen_origin = rotate(cv::Vec3d(0, 0, 0));
    for (int axis = 0; axis < 3; ++axis) {
      cv::Vec3d unit(0, 0, 0);
      unit[axis] = 1;
      const cv::Point2d direction = rotate(unit) - screen_origin;
      const cv::Point tip = axes_origin + cv::Point(direction * 32);
      cv::arrowedLine(scene, axes_origin, tip, {166, 169, 172}, 1, cv::LINE_AA, 0, 0.2);
      text(scene, std::string(1, "XYZ"[axis]), axes_origin + cv::Point(direction * 46), 0.42);
    }
  }

 private:
  const std::string title_;
  const size_t count_;
  const double initial_yaw = 0.92, initial_pitch = 0.52;
  cv::Vec3d pivot_;
  double yaw_ = initial_yaw, pitch_ = initial_pitch, zoom_ = 1.0, scale_ = 1.0;
  cv::Point last_cursor_;
  bool dragging_ = false;

  static void orbit(int event, int x, int y, int flags, void *context) {
    auto &view = *static_cast<OrbitView *>(context);
    const cv::Point cursor(x, y);
    if (event == cv::EVENT_LBUTTONDOWN && view.scene_.contains(cursor)) {
      view.dragging_ = true;
      view.last_cursor_ = cursor;
    } else if (event == cv::EVENT_MOUSEMOVE && view.dragging_ && (flags & cv::EVENT_FLAG_LBUTTON)) {
      const cv::Point delta = cursor - view.last_cursor_;
      view.last_cursor_ = cursor;
      view.yaw_ += 0.009 * delta.x;
      view.pitch_ = std::max(-1.55, std::min(1.55, view.pitch_ - 0.009 * delta.y));
      cv::imshow(view.title_, view.render());
    } else if (!(flags & cv::EVENT_FLAG_LBUTTON)) {
      view.dragging_ = false;
    }
  }
};

} // namespace lecture
