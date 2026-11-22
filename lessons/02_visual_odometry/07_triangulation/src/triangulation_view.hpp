#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include "image_panel.hpp"

struct Landmark {
  cv::Vec3d world;
  cv::Point2d pixel1, pixel2;
  double depth2;
};

class TriangulationView {
 public:
  TriangulationView(const cv::Mat &first, const cv::Mat &second, const std::vector<Landmark> &points,
                    const cv::Matx33d &K, const cv::Matx33d &R, const cv::Vec3d &t)
      : first_(first.clone()), second_(second.clone()), points_(points), invK_(K.inv()),
        camera2_rotation_(R.t()), camera2_center_(-(R.t() * t)) {
    cv::Vec3d low(0, 0, 0), high(0, 0, 0);
    for (const auto &p : points_) {
      cv::circle(first_, p.pixel1, 3, depth_color(p.world[2]), 2, cv::LINE_AA);
      cv::circle(second_, p.pixel2, 3, depth_color(p.depth2), 2, cv::LINE_AA);
      for (int k = 0; k < 3; ++k) {
        low[k] = std::min({low[k], p.world[k], camera2_center_[k]});
        high[k] = std::max({high[k], p.world[k], camera2_center_[k]});
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
    measure(cv::Vec3d(0, 0, 0));
    measure(camera2_center_);
    for (const auto &p : points_) measure(p.world);
    scale_ = std::min((scene_.width / 2 - 55) / dx, (scene_.height / 2 - 42) / dy);
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS")) {
      return cv::imwrite("view1.png", first_) && cv::imwrite("view2.png", second_) &&
             cv::imwrite("triangulation_overview.png", render()) ? 0 : 1;
    }
    cv::namedWindow(title_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(title_, 1160, 652);
    cv::setMouseCallback(title_, orbit, this);
    cv::imshow(title_, render());
    while (true) {
      const int key = cv::waitKey(40);
      if (key == 27 || key == 'q' || cv::getWindowProperty(title_, cv::WND_PROP_VISIBLE) == 0) break;
      switch (key) {
        case ' ': case 'n': selected_ = (selected_ + 1) % points_.size(); break;
        case 'p': selected_ = (selected_ + points_.size() - 1) % points_.size(); break;
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

 private:
  const std::string title_ = "Two-view triangulation";
  const cv::Rect scene_{12, 48, 762, 530};
  const cv::Scalar color1_{74, 199, 250}, color2_{219, 212, 80};
  const double initial_yaw = 0.92, initial_pitch = 0.52;
  cv::Mat first_, second_;
  const std::vector<Landmark> &points_;
  cv::Matx33d invK_, camera2_rotation_;
  cv::Vec3d camera2_center_, pivot_;
  double yaw_ = initial_yaw, pitch_ = initial_pitch, zoom_ = 1.0, scale_ = 1.0;
  size_t selected_ = 0;
  cv::Point last_cursor_;
  bool dragging_ = false;

  static cv::Scalar depth_color(double z) {
    const double f = std::max(0.0, std::min(1.0, (z - 10) / 40));
    return {80 + 160 * f, 106 - 30 * f, 250 - 131 * f};
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

  cv::Vec3d bearing(cv::Point2d pixel) const {
    return invK_ * cv::Vec3d(pixel.x, pixel.y, 1);
  }

  void camera(cv::Mat &scene, const cv::Vec3d &center, const cv::Matx33d &orientation,
              cv::Size size, cv::Scalar color) const {
    const double w = size.width - 1, h = size.height - 1;
    const cv::Point2d corners[] = {{0, 0}, {w, 0}, {w, h}, {0, h}};
    std::vector<cv::Point> rim;
    for (const auto &pixel : corners) {
      rim.push_back(project(center + orientation * bearing(pixel) * 0.70));
      cv::line(scene, project(center), rim.back(), color, 1, cv::LINE_AA);
    }
    cv::polylines(scene, rim, true, color, 1, cv::LINE_AA);
    cv::circle(scene, project(center), 4, color, -1, cv::LINE_AA);
  }

  static void text(cv::Mat &canvas, const std::string &s, cv::Point p, double size = 0.55) {
    cv::putText(canvas, s, p, cv::FONT_HERSHEY_SIMPLEX, size, {230, 235, 238}, 1, cv::LINE_AA);
  }

  cv::Mat render() const {
    cv::Mat canvas(652, 1160, CV_8UC3, cv::Scalar(32, 28, 24));
    cv::Mat scene = canvas(scene_);
    scene.setTo(cv::Scalar(46, 39, 32));
    const Landmark &p = points_[selected_];
    // Draw the measured image bearings to the estimated depth in each camera.
    const cv::Vec3d origin(0, 0, 0);
    const cv::Vec3d ray1 = bearing(p.pixel1) * p.world[2];
    const cv::Vec3d ray2 = camera2_center_ + camera2_rotation_ * bearing(p.pixel2) * p.depth2;
    cv::line(scene, project(origin), project(camera2_center_), {200, 200, 200}, 2, cv::LINE_AA);
    cv::line(scene, project(origin), project(ray1), color1_, 2, cv::LINE_AA);
    cv::line(scene, project(camera2_center_), project(ray2), color2_, 2, cv::LINE_AA);
    camera(scene, origin, cv::Matx33d::eye(), first_.size(), color1_);
    camera(scene, camera2_center_, camera2_rotation_, second_.size(), color2_);
    for (const auto &pt : points_)
      cv::circle(scene, project(pt.world), 3, depth_color(pt.world[2]), -1, cv::LINE_AA);
    cv::circle(scene, project(p.world), 9, {248, 248, 248}, 2, cv::LINE_AA);

    // Camera 1 axes use the same viewing rotation as the reconstruction.
    const cv::Point axes_origin(62, 58);
    const cv::Point2d screen_origin = rotate(origin);
    for (int axis = 0; axis < 3; ++axis) {
      cv::Vec3d unit(0, 0, 0);
      unit[axis] = 1;
      const cv::Point2d direction = rotate(unit) - screen_origin;
      const cv::Point tip = axes_origin + cv::Point(direction * 32);
      cv::arrowedLine(scene, axes_origin, tip, {166, 169, 172}, 1, cv::LINE_AA, 0, 0.2);
      text(scene, std::string(1, "XYZ"[axis]), axes_origin + cv::Point(direction * 46), 0.42);
    }

    cv::Mat first = first_.clone(), second = second_.clone();
    cv::circle(first, p.pixel1, 13, color1_, 4, cv::LINE_AA);
    cv::circle(second, p.pixel2, 13, color2_, 4, cv::LINE_AA);
    lecture::draw_panel(canvas, {790, 12, 356, 287}, first, "Camera 1");
    lecture::draw_panel(canvas, {790, 310, 356, 287}, second, "Camera 2");
    cv::circle(canvas, {1131, 28}, 5, color1_, -1, cv::LINE_AA);
    cv::circle(canvas, {1131, 326}, 5, color2_, -1, cv::LINE_AA);
    text(canvas, "Triangulated points", {12, 32}, 0.82);
    text(canvas, cv::format("Point %d / %d     Z1 = %.2f baselines",
                            int(selected_ + 1), int(points_.size()), p.world[2]), {18, 606}, 0.62);
    text(canvas, "Drag to rotate     +/- zoom     N/P select point     R reset     Q close", {18, 636});
    text(canvas, "Depth", {790, 626});
    text(canvas, "<=10", {863, 626});
    for (int x = 0; x <= 118; ++x)
      cv::line(canvas, {923 + x, 615}, {923 + x, 626}, depth_color(10.0 + 40.0 * x / 118));
    text(canvas, ">=50", {1052, 626});
    return canvas;
  }

  static void orbit(int event, int x, int y, int flags, void *context) {
    auto &view = *static_cast<TriangulationView *>(context);
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
