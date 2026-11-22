#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include "image_panel.hpp"
#include "orbit_view.hpp"

struct Landmark {
  cv::Vec3d world;
  cv::Point2d pixel1, pixel2;
  double depth2;
};

class TriangulationView : public lecture::OrbitView {
 public:
  TriangulationView(const cv::Mat &first, const cv::Mat &second, const std::vector<Landmark> &points,
                    const cv::Matx33d &K, const cv::Matx33d &R, const cv::Vec3d &t)
      : OrbitView("Two-view triangulation", points.size()), first_(first.clone()), second_(second.clone()),
        points_(points), invK_(K.inv()), camera2_rotation_(R.t()), camera2_center_(-(R.t() * t)) {
    std::vector<cv::Vec3d> positions{cv::Vec3d(0, 0, 0), camera2_center_};
    for (const auto &p : points_) {
      cv::circle(first_, p.pixel1, 3, depth_color(p.world[2]), 2, cv::LINE_AA);
      cv::circle(second_, p.pixel2, 3, depth_color(p.depth2), 2, cv::LINE_AA);
      positions.push_back(p.world);
    }
    fit(positions);
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS")) {
      return cv::imwrite("view1.png", first_) && cv::imwrite("view2.png", second_) &&
             cv::imwrite("triangulation_overview.png", render()) ? 0 : 1;
    }
    return show_window();
  }

 private:
  const cv::Scalar color1_{74, 199, 250}, color2_{219, 212, 80};
  cv::Mat first_, second_;
  const std::vector<Landmark> &points_;
  cv::Matx33d invK_, camera2_rotation_;
  cv::Vec3d camera2_center_;

  static cv::Scalar depth_color(double z) {
    const double f = std::max(0.0, std::min(1.0, (z - 10) / 40));
    return {80 + 160 * f, 106 - 30 * f, 250 - 131 * f};
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

  cv::Mat render() const override {
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
    draw_axes(scene);

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
};
