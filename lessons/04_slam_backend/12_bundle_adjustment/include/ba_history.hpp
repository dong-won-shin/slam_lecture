#pragma once

#include <ceres/solver.h>
#include <opencv2/calib3d.hpp>
#include "common.hpp"
#include "snavely_reprojection_error.hpp"

struct BACamera {
  cv::Vec3d center;
  cv::Matx33d orientation;
};

struct BAFrame {
  int iteration;
  double cost, rms;
  std::vector<cv::Vec3d> points;
  std::vector<BACamera> cameras;
  cv::Vec3d landmark;
  cv::Point2d residual;
};

class BAHistory : public ceres::IterationCallback {
 public:
  explicit BAHistory(const BALProblem &bal) : bal_(bal) {
    for (int i = 0; i < bal.num_observations(); ++i) {
      if (bal.point_index()[i] == bal.point_index()[0])
        viewing_cameras.push_back(bal.camera_index()[i]);
    }
    frames.push_back(snapshot(0, 0));
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary &step) override {
    if (step.iteration > 0 && step.step_is_successful)
      frames.push_back(snapshot(step.iteration, step.cost));
    return ceres::SOLVER_CONTINUE;
  }

  void finish(const ceres::Solver::Summary &summary) {
    frames.front().cost = summary.initial_cost;
    int last = frames.back().iteration;
    for (const auto &step : summary.iterations)
      if (step.step_is_successful) last = step.iteration;
    // Ceres can finish before invoking the last user callback.
    BAFrame final = snapshot(last, summary.final_cost);
    if (frames.back().iteration == last) frames.back() = std::move(final);
    else frames.push_back(std::move(final));
  }

  std::vector<BAFrame> frames;
  std::vector<int> viewing_cameras;

 private:
  const BALProblem &bal_;

  BAFrame snapshot(int iteration, double cost) const {
    BAFrame frame;
    frame.iteration = iteration;
    frame.cost = cost;
    // Keep the same point IDs in every frame, including for larger BAL datasets.
    const int stride = 1 + (bal_.num_points() - 1) / 12000;
    for (int i = 0; i < bal_.num_points(); i += stride)
      frame.points.emplace_back(bal_.points() + 3 * i);
    frame.landmark = cv::Vec3d(bal_.point_for_observation(0));

    const cv::Matx33d cv_to_bal(1, 0, 0, 0, -1, 0, 0, 0, -1);
    for (int i = 0; i < bal_.num_cameras(); ++i) {
      const double *camera = bal_.cameras() + i * bal_.camera_block_size();
      cv::Matx33d R;
      cv::Rodrigues(cv::Vec3d(camera), R);
      frame.cameras.push_back({-(R.t() * cv::Vec3d(camera + 3)), R.t() * cv_to_bal});
    }

    double sum_sq = 0;
    for (int i = 0; i < bal_.num_observations(); ++i) {
      double uv[2];
      SnavelyReprojectionError::CamProjectionWithDistortion(
          bal_.camera_for_observation(i), bal_.point_for_observation(i), uv);
      const cv::Point2d residual(uv[0] - bal_.observations()[2 * i],
                                 uv[1] - bal_.observations()[2 * i + 1]);
      sum_sq += residual.dot(residual);
      if (i == 0) frame.residual = residual;
    }
    frame.rms = std::sqrt(sum_sq / bal_.num_observations());
    return frame;
  }
};
