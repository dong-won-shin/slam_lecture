#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "epipolar_view.hpp"

static void find_feature_matches(const cv::Mat &first, const cv::Mat &second,
                                 std::vector<cv::Point2f> &points1,
                                 std::vector<cv::Point2f> &points2) {
  auto orb = cv::ORB::create();
  std::vector<cv::KeyPoint> keys1, keys2;
  cv::Mat descriptors1, descriptors2;
  orb->detectAndCompute(first, cv::noArray(), keys1, descriptors1);
  orb->detectAndCompute(second, cv::noArray(), keys2, descriptors2);
  if (descriptors1.empty() || descriptors2.empty()) return;

  std::vector<cv::DMatch> matches;
  cv::BFMatcher(cv::NORM_HAMMING).match(descriptors1, descriptors2, matches);
  const auto closest = std::min_element(matches.begin(), matches.end());
  const float threshold = std::max(2 * closest->distance, 30.0f);
  for (const auto &match : matches) {
    if (match.distance <= threshold) {
      points1.push_back(keys1[match.queryIdx].pt);
      points2.push_back(keys2[match.trainIdx].pt);
    }
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: pose_estimation_2d2d img1 img2\n";
    return 1;
  }
  cv::Mat first = cv::imread(argv[1]), second = cv::imread(argv[2]);
  if (first.empty() || second.empty()) {
    std::cerr << "Provide two readable images.\n";
    return 1;
  }

  std::vector<cv::Point2f> points1, points2;
  find_feature_matches(first, second, points1, points2);
  if (points1.size() < 8) {
    std::cerr << "Need at least eight matched features to estimate F.\n";
    return 1;
  }
  std::vector<uchar> mask;
  cv::Mat F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1.5, 0.99, mask);
  if (F.rows != 3 || F.cols != 3 || !cv::checkRange(F) || cv::countNonZero(mask) < 8) {
    std::cerr << "Could not estimate a fundamental matrix with enough inliers.\n";
    return 1;
  }
  std::cout << "fundamental_matrix is\n" << F << "\nRANSAC inliers: "
            << cv::countNonZero(mask) << " / " << points1.size() << '\n';

  // Camera calibration for the bundled RGB images.
  const cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  cv::Mat pose_mask, R, t;
  cv::Mat E = cv::findEssentialMat(points1, points2, K, cv::RANSAC, 0.999, 1.0, pose_mask);
  if (E.rows == 3 && E.cols == 3 && cv::recoverPose(E, points1, points2, K, R, t, pose_mask) > 0) {
    cv::Mat t_cross = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2), t.at<double>(1),
        t.at<double>(2), 0, -t.at<double>(0), -t.at<double>(1), t.at<double>(0), 0);
    std::cout << "essential_matrix is\n" << E << "\nR is\n" << R << "\nt is\n" << t
              << "\nt^R=\n" << t_cross * R << '\n';
  } else {
    std::cerr << "Could not recover a relative pose for this image pair.\n";
  }

  std::vector<cv::Point2f> inliers1, inliers2;
  for (size_t i = 0; i < mask.size(); ++i) {
    if (mask[i]) {
      inliers1.push_back(points1[i]);
      inliers2.push_back(points2[i]);
    }
  }
  EpipolarView view(first, second, F, inliers1, inliers2);
  return view.show();
}
