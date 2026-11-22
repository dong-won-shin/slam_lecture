#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/video/tracking.hpp>

#include "flow_stats.hpp"
#include "flow_example.hpp"

static cv::Mat render_lk(cv::Mat first, cv::Mat second, bool report) {
  cv::Mat gray1, gray2;
  cv::cvtColor(first, gray1, cv::COLOR_BGR2GRAY);
  cv::cvtColor(second, gray2, cv::COLOR_BGR2GRAY);

  // Detect corners only in frame 1. LK estimates their positions in frame 2.
  std::vector<cv::Point2f> corners, predicted, start, end, returned;
  cv::goodFeaturesToTrack(gray1, corners, 300, 0.01, 10.0);
  const cv::Size patch(21, 21);
  const int max_level = 3;
  const cv::Rect2f bounds(0, 0, first.cols, first.rows);
  std::vector<uchar> status;
  if (!corners.empty()) {
    cv::calcOpticalFlowPyrLK(gray1, gray2, corners, predicted, status, cv::noArray(),
                             patch, max_level);
  }
  for (size_t i = 0; i < corners.size(); ++i) {
    if (status[i] && bounds.contains(predicted[i])) {
      start.push_back(corners[i]);
      end.push_back(predicted[i]);
    }
  }
  // Track back to frame 1 and reject inconsistent correspondences.
  if (!end.empty()) {
    cv::calcOpticalFlowPyrLK(gray2, gray1, end, returned, status, cv::noArray(),
                             patch, max_level);
  }

  const cv::Scalar yellow(40, 220, 255);
  for (const auto &point : corners)
    cv::circle(first, point, 3, yellow, 1, cv::LINE_AA);
  std::vector<float> dx, dy;
  for (size_t i = 0; i < end.size(); ++i) {
    if (!status[i] || !bounds.contains(returned[i]) || cv::norm(returned[i] - start[i]) > 1.0)
      continue;
    dx.push_back(end[i].x - start[i].x);
    dy.push_back(end[i].y - start[i].y);
    cv::arrowedLine(second, start[i], end[i], yellow, 1, cv::LINE_AA, 0, 0.20);
    cv::circle(second, end[i], 2, cv::Scalar(40, 230, 100), -1, cv::LINE_AA);
  }
  if (report && dx.empty()) {
    std::cerr << "No tracks passed the forward-backward check.\n";
    return {};
  }

  if (report) {
    std::cout << "Detected: " << corners.size() << "\nTracked: " << dx.size()
              << std::fixed << std::setprecision(2)
              << "\nMedian flow (dx, dy) in pixels: " << flow::median(dx) << ", " << flow::median(dy) << '\n';
  }
  cv::Mat overview(424, 1040, CV_8UC3, cv::Scalar(29, 26, 24));
  lecture::draw_panel(overview, {12, 8, 502, 407}, first,
                      "Previous / detected corners (" + std::to_string(corners.size()) + ")");
  lecture::draw_panel(overview, {526, 8, 502, 407}, second,
                      "Current / tracked motion (" + std::to_string(dx.size()) + ")");
  return overview;
}

int main(int argc, char **argv) {
  return flow::run(argc, argv, "Pyramidal LK optical flow", "optical_flow.png", render_lk);
}
