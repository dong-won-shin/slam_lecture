#pragma once

#include <array>
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>
#include "image_panel.hpp"
#include "orbit_view.hpp"

struct PnPPose {
  const char *name;
  cv::Matx33d R;
  cv::Vec3d t;

  cv::Vec3d center() const { return -(R.t() * t); }
};

class PnPView : public lecture::OrbitView {
 public:
  PnPView(const cv::Mat &first, const cv::Mat &second, const cv::Mat &depth,
          double depth_scale, cv::Matx33d K, const std::vector<cv::Point3f> &points,
          const std::vector<cv::Point2f> &observations, const cv::Mat &inliers,
          const std::array<PnPPose, 3> &poses)
      : OrbitView("PnP camera pose", points.size()), second_(second), K_(K), invK_(K.inv()),
        first_size_(first.size()), points_(points), observations_(observations), poses_(poses),
        inliers_(points.size(), false), inlier_count_(inliers.total()) {
    std::vector<cv::Vec3d> bounds{cv::Vec3d(0, 0, 0)};
    for (const auto &p : points_) bounds.push_back(cv::Vec3d(p.x, p.y, p.z));
    for (const auto &pose : poses_) bounds.push_back(pose.center());
    fit(bounds);
    for (size_t i = 0; i < inliers.total(); ++i) inliers_[inliers.at<int>(int(i))] = true;
    // This subsampled RGB-D cloud provides scene context only. PnP uses the matched points.
    for (int y = 0; y < depth.rows; y += 5) {
      for (int x = 0; x < depth.cols; x += 5) {
        const double z = depth.at<unsigned short>(y, x) / depth_scale;
        if (z > 0) cloud_.push_back({(invK_ * cv::Vec3d(x, y, 1)) * z, first.at<cv::Vec3b>(y, x)});
      }
    }
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS")) return cv::imwrite("pnp_pose.png", render()) ? 0 : 1;
    return show_window();
  }

 private:
  struct Sample { cv::Vec3d position; cv::Vec3b color; };
  const cv::Mat &second_;
  cv::Matx33d K_, invK_;
  cv::Size first_size_;
  const std::vector<cv::Point3f> &points_;
  const std::vector<cv::Point2f> &observations_;
  std::array<PnPPose, 3> poses_;
  std::vector<bool> inliers_;
  size_t inlier_count_;
  std::vector<Sample> cloud_;
  int method_ = 0;
  bool show_cloud_ = true;
  const cv::Scalar gold_{60, 197, 255}, cyan_{230, 213, 75};

  bool handle_key(int key) override {
    if (key >= '1' && key <= '3') method_ = key - '1';
    else if (key == 'c') show_cloud_ = !show_cloud_;
    else return false;
    return true;
  }

  bool reproject(const PnPPose &pose, const cv::Vec3d &world, cv::Point2d &pixel) const {
    const cv::Vec3d camera = pose.R * world + pose.t;
    if (camera[2] <= 0) return false;
    const cv::Vec3d uv = K_ * camera;
    pixel = {uv[0] / uv[2], uv[1] / uv[2]};
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
  }

  cv::Mat render() const override {
    cv::Mat canvas(652, 1160, CV_8UC3, cv::Scalar(32, 28, 24));
    cv::Mat scene = canvas(scene_);
    scene.setTo(cv::Scalar(46, 39, 32));
    if (show_cloud_) {
      for (const auto &sample : cloud_) {
        const auto c = sample.color;
        cv::circle(scene, project(sample.position), 1, cv::Scalar(c[0], c[1], c[2]) * 0.57, -1);
      }
    }
    const auto &pose = poses_[method_];
    const auto &p = points_[selected_];
    const cv::Vec3d world(p.x, p.y, p.z), origin(0, 0, 0), center = pose.center();
    for (const auto &pt : points_)
      cv::circle(scene, project(cv::Vec3d(pt.x, pt.y, pt.z)), 3, {170, 175, 185}, -1, cv::LINE_AA);
    cv::line(scene, project(origin), project(center), {200, 200, 200}, 1, cv::LINE_AA);
    cv::line(scene, project(center), project(world), cyan_, 1, cv::LINE_AA);
    draw_camera(scene, origin, cv::Matx33d::eye(), first_size_, invK_, 0.12, gold_);
    draw_camera(scene, center, pose.R.t(), second_.size(), invK_, 0.12, cyan_);
    text(scene, "C1", project(origin) + cv::Point(-30, -8), 0.5);
    text(scene, "C2", project(center) + cv::Point(8, 18), 0.5);
    cv::circle(scene, project(world), 8, {248, 248, 248}, 2, cv::LINE_AA);
    draw_axes(scene);

    const cv::Point2d obs = observations_[selected_];
    cv::Point2d predicted;
    const bool visible = reproject(pose, world, predicted);
    cv::Mat photo = second_.clone();
    cv::circle(photo, obs, 11, gold_, 3, cv::LINE_AA);
    if (visible && cv::Rect2d(0, 0, photo.cols, photo.rows).contains(predicted))
      cv::drawMarker(photo, predicted, cyan_, cv::MARKER_CROSS, 15, 3, cv::LINE_AA);
    lecture::draw_panel(canvas, {790, 12, 356, 287}, photo, "Camera 2  /  reprojection");

    // Enlarge a patch around the subpixel observation to make small residuals visible.
    cv::Mat patch;
    cv::getRectSubPix(second_, {25, 25}, obs, patch);
    cv::Mat detail = canvas(cv::Rect(796, 355, 125, 125));
    cv::resize(patch, detail, detail.size(), 0, 0, cv::INTER_NEAREST);
    const cv::Point mid(62, 62);
    cv::circle(detail, mid, 8, gold_, 2, cv::LINE_AA);
    const cv::Point2d offset = (predicted - obs) * 5;
    if (visible && std::abs(offset.x) <= 62 && std::abs(offset.y) <= 62)
      cv::drawMarker(detail, mid + cv::Point(offset), cyan_, cv::MARKER_CROSS, 13, 2, cv::LINE_AA);
    cv::circle(canvas, {802, 323}, 6, gold_, 2, cv::LINE_AA);
    text(canvas, "Observed", {818, 329});
    cv::drawMarker(canvas, {953, 323}, cyan_, cv::MARKER_CROSS, 13, 2);
    text(canvas, "Projected", {969, 329});
    text(canvas, "5x detail", {796, 503});
    text(canvas, "Pixel error", {939, 369});
    for (size_t i = 0; i < poses_.size(); ++i) {
      cv::Point2d uv;
      const auto error = reproject(poses_[i], world, uv)
          ? cv::format("%.2f px", cv::norm(uv - obs)) : std::string("behind");
      text(canvas, poses_[i].name, {939, 399 + int(i) * 35}, 0.48);
      text(canvas, error, {1066, 399 + int(i) * 35}, 0.48);
    }
    text(canvas, "C2 center in camera 1 (m)", {796, 546});
    text(canvas, cv::format("XYZ    %.3f    %.3f    %.3f", center[0], center[1], center[2]), {796, 576});
    text(canvas, "PnP camera pose", {12, 32}, 0.82);
    text(canvas, cv::format("%s   |   Point %d / %d   |   RANSAC %s", pose.name, int(selected_ + 1),
                            int(points_.size()), inliers_[selected_] ? "inlier" : "outlier"), {18, 606}, 0.63);
    text(canvas, "Drag rotate    +/- zoom    N/P point    1/2/3 solver    C cloud    R reset    Q close", {18, 635}, 0.50);
    text(canvas, cv::format("RANSAC inliers   %d / %d", int(inlier_count_), int(points_.size())), {796, 621});
    return canvas;
  }
};
