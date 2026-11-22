#pragma once

#include <cstdlib>
#include <opencv2/imgcodecs.hpp>
#include "orbit_view.hpp"

struct OpenGVPoint {
  cv::Vec3d truth, linear, midpoint;
  cv::Vec3d bearing1, bearing2;  // Both unit directions in camera 1 coordinates.
};

class OpenGVView : public lecture::OrbitView {
 public:
  OpenGVView(const std::vector<OpenGVPoint> &points, const cv::Vec3d &center2)
      : OrbitView("OpenGV triangulation", points.size()), points_(points), center2_(center2) {
    std::vector<cv::Vec3d> positions{cv::Vec3d(0, 0, 0), center2};
    for (const auto &p : points_) {
      positions.insert(positions.end(), {p.truth, p.linear, p.midpoint});
    }
    fit(positions);
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS"))
      return cv::imwrite("opengv_triangulation.png", render()) ? 0 : 1;
    return show_window();
  }

 private:
  const std::vector<OpenGVPoint> &points_;
  const cv::Vec3d center2_;
  const cv::Scalar linear_color_{90, 170, 255}, midpoint_color_{236, 204, 75};

  void markers(cv::Mat &img, cv::Point truth, cv::Point linear, cv::Point midpoint, int size) const {
    cv::circle(img, truth, size, {235, 235, 235}, 1, cv::LINE_AA);
    cv::drawMarker(img, linear, linear_color_, cv::MARKER_CROSS, size + 3, 1, cv::LINE_AA);
    cv::drawMarker(img, midpoint, midpoint_color_, cv::MARKER_TILTED_CROSS, size + 3, 1, cv::LINE_AA);
  }

  cv::Mat render() const override {
    cv::Mat canvas(652, 1160, CV_8UC3, cv::Scalar(32, 28, 24));
    cv::Mat scene = canvas(scene_);
    scene.setTo(cv::Scalar(46, 39, 32));
    const auto &p = points_[selected_];
    const cv::Vec3d origin(0, 0, 0);
    // Extend the measured rays to the perpendicular projection of the true point.
    const cv::Vec3d end1 = p.bearing1 * p.truth.dot(p.bearing1);
    const cv::Vec3d end2 = center2_ + p.bearing2 * (p.truth - center2_).dot(p.bearing2);
    cv::line(scene, project(origin), project(center2_), {180, 180, 180}, 1, cv::LINE_AA);
    cv::line(scene, project(origin), project(end1), {105, 199, 160}, 2, cv::LINE_AA);
    cv::line(scene, project(center2_), project(end2), {200, 165, 170}, 2, cv::LINE_AA);
    for (const auto &point : points_) {
      markers(scene, project(point.truth), project(point.linear), project(point.midpoint), 5);
    }
    cv::circle(scene, project(origin), 5, {105, 199, 160}, -1, cv::LINE_AA);
    cv::circle(scene, project(center2_), 5, {200, 165, 170}, -1, cv::LINE_AA);
    text(scene, "C1", project(origin) + cv::Point(-30, -8));
    text(scene, "C2", project(center2_) + cv::Point(8, 18));
    cv::circle(scene, project(p.truth), 11, {240, 240, 240}, 2, cv::LINE_AA);
    draw_axes(scene);

    text(canvas, "OpenGV triangulation", {12, 32}, 0.82);
    text(canvas, cv::format("Point %d / %d", int(selected_ + 1), int(points_.size())), {18, 606}, 0.64);
    text(canvas, "Drag to rotate     +/- zoom     N/P select point     R reset     Q close", {18, 636});
    text(canvas, "Selected point", {794, 32}, 0.76);
    const cv::Vec3d positions[] = {p.truth, p.linear, p.midpoint};
    const char *names[] = {"Ground truth", "triangulate  /  linear SVD", "triangulate2  /  midpoint"};
    for (int i = 0; i < 3; ++i) {
      const int y = 80 + i * 96;
      const cv::Point symbol(808, y - 5);
      if (i == 0) cv::circle(canvas, symbol, 7, {235, 235, 235}, 1, cv::LINE_AA);
      else cv::drawMarker(canvas, symbol, i == 1 ? linear_color_ : midpoint_color_,
                          i == 1 ? cv::MARKER_CROSS : cv::MARKER_TILTED_CROSS, 13, 2);
      text(canvas, names[i], {829, y}, 0.60);
      const auto &v = positions[i];
      text(canvas, cv::format("XYZ   % .4f   % .4f   % .4f", v[0], v[1], v[2]), {796, y + 27}, 0.50);
      if (i) text(canvas, cv::format("3D error   %.6f", cv::norm(v - p.truth)), {796, y + 51}, 0.51);
    }

    // Magnify only this inset. Its origin is the true point; its orientation follows the scene.
    const double span = std::max(cv::norm(p.linear - p.truth), cv::norm(p.midpoint - p.truth));
    const double units_per_pixel = std::max(span, 1e-12) / 90.0;
    cv::Mat detail = canvas(cv::Rect(794, 380, 350, 204));
    detail.setTo(cv::Scalar(46, 39, 32));
    const cv::Point center(175, 102);
    auto detail_point = [&](const cv::Vec3d &v) {
      return center + cv::Point((rotate(v) - rotate(p.truth)) / units_per_pixel);
    };
    text(canvas, "Error detail around ground truth", {794, 365}, 0.61);
    const auto a = detail_point(p.linear), b = detail_point(p.midpoint);
    cv::line(detail, center, a, linear_color_, 1, cv::LINE_AA);
    cv::line(detail, center, b, midpoint_color_, 1, cv::LINE_AA);
    markers(detail, center, a, b, 9);
    cv::line(canvas, {796, 610}, {856, 610}, {230, 230, 230}, 2);
    text(canvas, cv::format("%.5g scene units", 60 * units_per_pixel), {868, 616}, 0.51);
    return canvas;
  }
};
