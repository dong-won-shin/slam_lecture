#pragma once

#include <cstdlib>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include "orbit_view.hpp"

struct PoseGraphState {
  std::vector<cv::Vec3d> positions;
  double chi2;
};

struct PoseConstraint {
  size_t from, to;
  bool consecutive;
};

class PoseGraphView : public lecture::OrbitView {
 public:
  PoseGraphView(const std::vector<PoseGraphState> &states,
                const std::vector<PoseConstraint> &edges, const std::vector<size_t> &anchors,
                bool trajectory = false)
      : OrbitView("SE3 pose graph", states.size(), WorldY::Up, {12, 88, 562, 432},
                  trajectory ? cv::Point2d(0.1, 1.15) : cv::Point2d(0.92, 0.52)),
        states_(states), edges_(edges), anchors_(anchors), trajectory_(trajectory) {
    selected_ = states_.size() - 1;
    std::vector<cv::Vec3d> bounds;
    for (const auto &state : states_)
      bounds.insert(bounds.end(), state.positions.begin(), state.positions.end());
    fit(bounds);
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS"))
      return cv::imwrite("pose_graph.png", render()) ? 0 : 1;
    return show_window();
  }

 private:
  const std::vector<PoseGraphState> &states_;
  const std::vector<PoseConstraint> &edges_;
  const std::vector<size_t> &anchors_;
  const bool trajectory_;
  bool show_cross_edges_ = true;

  cv::Scalar cross_color() const {
    return trajectory_ ? cv::Scalar(130, 131, 247) : cv::Scalar(152, 125, 99);
  }

  cv::Rect right_panel() const { return scene_ + cv::Point(574, 0); }
  cv::Rect orbit_area() const override { return scene_ | right_panel(); }

  bool handle_key(int key) override {
    if (key == '0') selected_ = 0;
    else if (key == 'e') selected_ = states_.size() - 1;
    else if (key == 'c') show_cross_edges_ = !show_cross_edges_;
    else return false;
    return true;
  }

  void draw_graph(cv::Mat panel, const PoseGraphState &state) const {
    panel.setTo(cv::Scalar(43, 36, 29));
    std::vector<cv::Point> pixels;
    std::vector<double> depth;
    for (const auto &position : state.positions) {
      pixels.push_back(project(position));
      depth.push_back(view_coordinates(position)[2]);
    }
    const auto extent = std::minmax_element(depth.begin(), depth.end());
    const double back = *extent.second, span = std::max(1.e-10, back - *extent.first);
    std::vector<size_t> order(edges_.size());
    std::iota(order.begin(), order.end(), 0);
    auto edge_depth = [&](const PoseConstraint &e) { return (depth[e.from] + depth[e.to]) * 0.5; };
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return edge_depth(edges_[a]) > edge_depth(edges_[b]);
    });
    for (size_t index : order) {
      const auto &edge = edges_[index];
      if (!edge.consecutive && !show_cross_edges_) continue;
      const double light = trajectory_ ? 1.0 : 0.30 + 0.70 * (back - edge_depth(edge)) / span;
      const cv::Scalar color = edge.consecutive ? cv::Scalar(194, 226, 83) : cross_color();
      cv::line(panel, pixels[edge.from], pixels[edge.to], color * light, trajectory_ ? 2 : 1, cv::LINE_AA);
    }
    for (size_t i = 0; i < pixels.size(); ++i) {
      const double light = 0.35 + 0.65 * (back - depth[i]) / span;
      cv::circle(panel, pixels[i], 1, cv::Scalar(210, 229, 177) * light, -1);
    }
    if (trajectory_ && show_cross_edges_) {
      for (const auto &edge : edges_) {
        if (edge.consecutive) continue;
        for (size_t i : {edge.from, edge.to})
          cv::circle(panel, pixels[i], 4, cross_color(), 1, cv::LINE_AA);
      }
    }
    for (size_t anchor : anchors_)
      cv::circle(panel, pixels[anchor], 6, {65, 193, 255}, 2, cv::LINE_AA);
    if (trajectory_) {
      cv::circle(panel, pixels.front(), 9, {65, 193, 255}, 2, cv::LINE_AA);
      cv::drawMarker(panel, pixels.back(), {238, 238, 238}, cv::MARKER_CROSS, 11, 2, cv::LINE_AA);
    }
    draw_axes(panel);
  }

  cv::Mat render() const override {
    const auto &initial = states_.front(), &current = states_[selected_];
    cv::Mat canvas(652, 1160, CV_8UC3, cv::Scalar(32, 28, 24));
    text(canvas, trajectory_ ? "Driving trajectory" : "SE3 pose graph", {14, 34}, 0.84);
    text(canvas, cv::format("%zu poses   /   %zu constraints", initial.positions.size(), edges_.size()),
         {748, 33}, 0.60);
    text(canvas, "Initial estimates", {16, 73}, 0.66);
    text(canvas, cv::format("Iteration %zu / %zu", selected_, states_.size() - 1), {590, 73}, 0.66);
    draw_graph(canvas(scene_), initial);
    draw_graph(canvas(right_panel()), current);
    for (const auto &area : {scene_, right_panel()}) {
      const auto &state = area == scene_ ? initial : current;
      text(canvas, cv::format("Chi-squared   %.2f", state.chi2), {area.x + 4, 552}, 0.57);
      if (trajectory_)
        text(canvas, cv::format("Start to end   %.2f m", cv::norm(state.positions.back() - state.positions.front())),
             {area.x + 280, 552}, 0.57);
    }
    cv::line(canvas, {18, 584}, {44, 584}, {194, 226, 83}, 2);
    text(canvas, trajectory_ ? "Odometry" : "Consecutive pose IDs", {54, 590});
    cv::line(canvas, {325, 584}, {351, 584}, cross_color(), 2);
    std::string edge_label = trajectory_ ? "Revisit constraints" : "Other constraints";
    if (!show_cross_edges_) edge_label += " (hidden)";
    text(canvas, edge_label, {361, 590});
    cv::circle(canvas, {739, 584}, 6, {65, 193, 255}, 2, cv::LINE_AA);
    text(canvas, trajectory_ ? "Start" : "Fixed pose", {755, 590});
    if (trajectory_) {
      cv::drawMarker(canvas, {907, 584}, {238, 238, 238}, cv::MARKER_CROSS, 11, 2);
      text(canvas, "End", {925, 590});
    }
    text(canvas, "Drag rotate    +/- zoom    N/P step    0/E first/last    C other constraints    R reset view    Q close",
         {16, 634}, 0.54);
    return canvas;
  }
};
