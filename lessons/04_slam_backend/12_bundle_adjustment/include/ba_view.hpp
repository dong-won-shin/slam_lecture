#pragma once

#include <chrono>
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include "ba_history.hpp"
#include "orbit_view.hpp"

class BAView : public lecture::OrbitView {
 public:
  BAView(const BAHistory &history, const BALProblem &bal)
      : OrbitView("Bundle adjustment", history.frames.size()), history_(history), bal_(bal) {
    const auto &initial = history.frames.front();
    std::vector<cv::Vec3d> bounds;
    // Frame the central cloud and all camera centers; distant landmarks still render.
    cv::Vec3d low, high;
    for (int k = 0; k < 3; ++k) {
      std::vector<double> values;
      for (const auto &point : initial.points) values.push_back(point[k]);
      std::sort(values.begin(), values.end());
      low[k] = values[(values.size() - 1) / 100];
      high[k] = values[values.size() - 1 - (values.size() - 1) / 100];
    }
    for (int corner = 0; corner < 8; ++corner)
      bounds.emplace_back(corner & 1 ? high[0] : low[0], corner & 2 ? high[1] : low[1],
                          corner & 4 ? high[2] : low[2]);
    for (const auto &frame : history.frames)
      for (const auto &camera : frame.cameras) bounds.push_back(camera.center);
    fit(bounds);
    camera_size_ = std::max(0.001, cv::norm(high - low) * 0.035);
  }

  int show() {
    if (!std::getenv("SLAM_HEADLESS")) return show_window();
    cv::VideoWriter movie("ba_iterations.mp4", cv::VideoWriter::fourcc('m','p','4','v'),
                          2.0, {1160, 652});
    if (!movie.isOpened()) {
      std::cerr << "Could not create ba_iterations.mp4\n";
      return 1;
    }
    for (selected_ = 0; selected_ < history_.frames.size(); ++selected_) {
      const cv::Mat frame = render();
      movie.write(frame);
      if (selected_ + 1 == history_.frames.size() && !cv::imwrite("ba_final.png", frame)) return 1;
    }
    return 0;
  }

 private:
  const BAHistory &history_;
  const BALProblem &bal_;
  double camera_size_;
  bool playing_ = false;
  std::chrono::steady_clock::time_point last_tick_;

  bool handle_key(int key) override {
    const size_t last = history_.frames.size() - 1;
    const auto now = std::chrono::steady_clock::now();
    if (key == ' ') {
      if (!playing_ && selected_ == last) selected_ = 0;
      playing_ = !playing_ && last > 0;
      last_tick_ = now;
    } else if (key == '0' || key == 'e' || key == 'p' || key == 'n') {
      playing_ = false;
      if (key == '0') selected_ = 0;
      if (key == 'e') selected_ = last;
      if (key == 'p' && selected_) --selected_;
      if (key == 'n' && selected_ < last) ++selected_;
    } else if (key == -1 && playing_ && now - last_tick_ >= std::chrono::milliseconds(500)) {
      ++selected_;
      playing_ = selected_ < last;
      last_tick_ = now;
    } else return false;
    return true;
  }

  void cost_plot(cv::Mat &canvas) const {
    const auto &frames = history_.frames;
    const double top = std::log1p(frames.front().cost), bottom = std::log1p(frames.back().cost);
    auto chart_point = [&](size_t i) {
      const double fraction = (top - std::log1p(frames[i].cost)) / std::max(1e-12, top - bottom);
      return cv::Point(866 + cvRound(250.0 * frames[i].iteration / std::max(1, frames.back().iteration)),
                       137 + cvRound(120 * fraction));
    };
    text(canvas, "Huber cost   /   log(1 + cost)", {794, 103}, 0.60);
    for (int y : {137, 197, 257})
      cv::line(canvas, {866, y}, {1116, y}, {74, 66, 61}, 1);
    for (size_t i = 1; i < frames.size(); ++i)
      cv::line(canvas, chart_point(i-1), chart_point(i),
               i <= selected_ ? cv::Scalar(232, 211, 80) : cv::Scalar(111, 104, 95), 2, cv::LINE_AA);
    cv::circle(canvas, chart_point(selected_), 5, {232, 211, 80}, -1, cv::LINE_AA);
    text(canvas, cv::format("%.2g", frames.front().cost), {791, 141}, 0.43);
    text(canvas, cv::format("%.2g", frames.back().cost), {791, 261}, 0.43);
    text(canvas, "0", {860, 280}, 0.45);
    text(canvas, "Ceres iteration", {900, 280}, 0.47);
    text(canvas, std::to_string(frames.back().iteration), {1100, 280}, 0.45);
  }

  cv::Mat render() const override {
    const auto &frame = history_.frames[selected_];
    cv::Mat canvas(652, 1160, CV_8UC3, cv::Scalar(32, 28, 24));
    cv::Mat scene = canvas(scene_);
    scene.setTo(cv::Scalar(46, 39, 32));
    for (const auto &point : frame.points)
      cv::circle(scene, project(point), 1, {184, 179, 167}, -1);
    for (int id : history_.viewing_cameras)
      cv::line(scene, project(frame.cameras[id].center), project(frame.landmark),
               {91, 145, 153}, 1, cv::LINE_AA);
    const int camera_id = bal_.camera_index()[0];
    const cv::Matx33d symbol(1,0,-1, 0,1,-1, 0,0,1);
    auto glyph = [&](int i, cv::Scalar color) {
      const auto &camera = frame.cameras[i];
      draw_camera(scene, camera.center, camera.orientation, {3,3}, symbol, camera_size_,
                  color);
    };
    for (int i = 0; i < bal_.num_cameras(); ++i)
      if (i != camera_id) glyph(i, {216,202,72});
    glyph(camera_id, {55,199,255});
    cv::circle(scene, project(frame.landmark), 7, {55,199,255}, 2, cv::LINE_AA);
    draw_axes(scene);
    text(canvas, "Bundle adjustment", {12,32}, 0.82);
    text(canvas, cv::format("%d cameras   /   %d landmarks", bal_.num_cameras(), bal_.num_points()),
         {18,606}, 0.64);
    text(canvas, "Drag rotate    +/- zoom    Space play/pause    N/P step    0/E first/last    Q close",
         {18,636}, 0.50);
    text(canvas, cv::format("Iteration %d", frame.iteration), {794,32}, 0.82);
    text(canvas, cv::format("State %d of %d", int(selected_ + 1), int(history_.frames.size())), {795,64});
    cost_plot(canvas);
    text(canvas, "Reprojection across all observations", {794,335}, 0.55);
    text(canvas, cv::format("RMS    %.2f px", frame.rms), {794,380}, 0.81);
    text(canvas, cv::format("Initial RMS    %.2f px", history_.frames.front().rms), {794,413});
    text(canvas, cv::format("Huber cost    %.1f", frame.cost), {794,446});
    text(canvas, cv::format("Point %d in camera %d", bal_.point_index()[0], camera_id), {794,519}, 0.67);
    text(canvas, cv::format("Pixel error    %.2f px", cv::norm(frame.residual)), {794,554});
    text(canvas, cv::format("Observed by %d cameras", int(history_.viewing_cameras.size())), {794,588});
    return canvas;
  }
};
