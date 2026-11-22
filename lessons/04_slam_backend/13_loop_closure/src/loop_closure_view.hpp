#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>
#include "DBoW3/DBoW3.h"
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include "image_panel.hpp"

struct LoopCandidate {
  static constexpr int min_inliers = 15;
  DBoW3::Result retrieval;
  std::vector<cv::DMatch> matches, inliers;

  bool verified() const {
    return inliers.size() >= min_inliers && inliers.size() * 2 >= matches.size();
  }
};

struct LoopFrame {
  cv::Mat image, descriptors;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<LoopCandidate> candidates;

  int best_verified_rank() const {
    for (size_t i = 0; i < candidates.size(); ++i)
      if (candidates[i].verified()) return static_cast<int>(i);
    return -1;
  }
};

class LoopClosureView {
 public:
  LoopClosureView(const std::vector<LoopFrame> &frames, size_t initial_query)
      : frames_(frames), query_(initial_query), rank_(frames[initial_query].best_verified_rank()) {
    // Keep a stable layout when switching queries, including panoramic driving images.
    for (const auto &frame : frames) {
      const auto &im = frame.image;
      photo_height_ = std::max(photo_height_, std::min(405, cvRound(554.0 * im.rows / im.cols)));
    }
  }

  int show() {
    if (std::getenv("SLAM_HEADLESS")) {
      if (!cv::imwrite("loop_closure.png", render())) return 1;
      show_matches_ = true;
      return cv::imwrite("loop_closure_matches.png", render()) ? 0 : 1;
    }
    cv::namedWindow(title_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(title_, canvas_size());
    cv::setMouseCallback(title_, on_click, this);
    cv::imshow(title_, render());
    while (true) {
      const int key = cv::waitKey(50);
      if (key == 'q' || key == 27 || cv::getWindowProperty(title_, cv::WND_PROP_VISIBLE) == 0) break;
      if (key == 'n' || key == 'p') {
        query_ = (query_ + (key == 'n' ? 1 : frames_.size() - 1)) % frames_.size();
        rank_ = frames_[query_].best_verified_rank();
      } else if (key >= '1' && key <= '3' && size_t(key - '1') < frames_[query_].candidates.size()) {
        rank_ = key - '1';
      } else if (key == 'm') {
        show_matches_ = !show_matches_;
      } else continue;
      cv::imshow(title_, render());
    }
    cv::destroyAllWindows();
    return 0;
  }

 private:
  const std::string title_ = "Loop closure candidates";
  const std::vector<LoopFrame> &frames_;
  size_t query_;
  int rank_;
  int photo_height_ = 1;
  bool show_matches_ = false;
  const cv::Scalar ink_{235, 238, 242}, muted_{182, 178, 168}, accent_{105, 211, 247};

  cv::Rect card(size_t rank) const { return {16 + 380 * int(rank), photo_height_ + 89, 370, 145}; }
  cv::Size canvas_size() const { return {1160, card(0).br().y + 52}; }

  static void label(cv::Mat &img, const std::string &s, cv::Point at,
                    double scale, cv::Scalar color) {
    cv::putText(img, s, at, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
  }

  static cv::Point pixel_on_canvas(cv::Point2f pt, cv::Size size, cv::Rect area) {
    return {area.x + cvRound((pt.x + 0.5) * area.width / size.width - 0.5),
            area.y + cvRound((pt.y + 0.5) * area.height / size.height - 0.5)};
  }

  static void draw_matches(cv::Mat &canvas, const LoopFrame &a, const LoopFrame &b,
                           cv::Rect left, cv::Rect right, const std::vector<cv::DMatch> &matches) {
    const cv::Scalar colors[] = {{76, 222, 255}, {244, 218, 95}, {184, 152, 255}};
    for (size_t i = 0; i < std::min<size_t>(20, matches.size()); ++i) {
      const auto &m = matches[i];
      auto p = pixel_on_canvas(a.keypoints[m.queryIdx].pt, a.image.size(), left);
      auto q = pixel_on_canvas(b.keypoints[m.trainIdx].pt, b.image.size(), right);
      auto color = colors[i % 3];
      cv::line(canvas, p, q, {35, 34, 29}, 3, cv::LINE_AA);
      cv::line(canvas, p, q, color, 1, cv::LINE_AA);
      cv::circle(canvas, p, 4, color, 2, cv::LINE_AA);
      cv::circle(canvas, q, 4, color, 2, cv::LINE_AA);
    }
  }

  cv::Mat render() const {
    cv::Mat canvas(canvas_size(), CV_8UC3, cv::Scalar(33, 29, 25));
    const auto &query = frames_[query_];
    label(canvas, cv::format("Query   /   %d.png", int(query_ + 1)), {19, 33}, 0.79, ink_);
    auto left = lecture::draw_image(canvas, {16, 46, 554, photo_height_}, query.image);
    if (rank_ >= 0) {
      const auto &result = query.candidates[rank_];
      const auto &candidate = frames_[result.retrieval.Id];
      label(canvas, cv::format("%s   /   %u.png", result.verified() ? "Verified" : "Rejected", result.retrieval.Id + 1), {591, 33}, 0.79, ink_);
      auto right = lecture::draw_image(canvas, {590, 46, 554, photo_height_}, candidate.image);
      const auto evidence = result.matches.size() < LoopCandidate::min_inliers
          ? cv::format("%d matches / %d required", int(result.matches.size()), LoopCandidate::min_inliers)
          : cv::format("%d / %d geometric inliers", int(result.inliers.size()), int(result.matches.size()));
      label(canvas, evidence, {591, photo_height_ + 75}, 0.54, muted_);
      if (show_matches_) draw_matches(canvas, query, candidate, left, right, result.inliers);
    } else {
      const bool empty = query.candidates.empty();
      label(canvas, empty ? "No earlier candidate" : "No verified loop", {680, 46 + photo_height_ / 2}, 0.76, ink_);
      if (!empty)
        label(canvas, "Select a ranked candidate below to inspect it.", {660, 80 + photo_height_ / 2}, 0.53, muted_);
    }
    label(canvas, "Earlier images ranked by DBoW3", {19, photo_height_ + 75}, 0.62, ink_);
    for (size_t i = 0; i < query.candidates.size(); ++i) {
      const auto &r = query.candidates[i];
      const auto box = card(i);
      canvas(box).setTo(cv::Scalar(52, 46, 40));
      lecture::draw_image(canvas, {box.x + 5, box.y + 5, box.width - 10, 104}, frames_[r.retrieval.Id].image);
      label(canvas, cv::format("#%d  %u.png", int(i + 1), r.retrieval.Id + 1), box.tl() + cv::Point(12, 134), 0.62, ink_);
      label(canvas, cv::format("Score   %.4f", r.retrieval.Score), box.tl() + cv::Point(206, 134), 0.54, muted_);
      label(canvas, r.verified() ? "Verified" : "Rejected", box.tl() + cv::Point(133, 134),
            0.45, r.verified() ? accent_ : muted_);
      if (int(i) == rank_) cv::rectangle(canvas, box, accent_, 2, cv::LINE_AA);
    }
    label(canvas, cv::format("N / P   change query       1 - 3 or click   select candidate       M   inliers %s       Q   close",
                             show_matches_ ? "on" : "off"),
          {19, canvas.rows - 19}, 0.55, muted_);
    return canvas;
  }

  static void on_click(int event, int x, int y, int, void *context) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto &view = *static_cast<LoopClosureView *>(context);
    for (size_t i = 0; i < view.frames_[view.query_].candidates.size(); ++i) {
      if (view.card(i).contains(cv::Point(x, y))) {
        view.rank_ = static_cast<int>(i);
        cv::imshow(view.title_, view.render());
        return;
      }
    }
  }
};
