#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "triangulation_view.hpp"

using namespace cv;
using namespace std;

static void match_features(const Mat &first, const Mat &second, vector<Point2d> &pixels1,
                           vector<Point2d> &pixels2) {
  auto orb = ORB::create();
  vector<KeyPoint> keys1, keys2;
  Mat descriptors1, descriptors2;
  orb->detectAndCompute(first, noArray(), keys1, descriptors1);
  orb->detectAndCompute(second, noArray(), keys2, descriptors2);
  if (descriptors1.empty() || descriptors2.empty()) return;
  vector<DMatch> matches;
  BFMatcher(NORM_HAMMING).match(descriptors1, descriptors2, matches);
  const float threshold = max(30.0f, 2 * min_element(matches.begin(), matches.end())->distance);
  for (const auto &m : matches) {
    if (m.distance <= threshold) {
      pixels1.push_back(keys1[m.queryIdx].pt);
      pixels2.push_back(keys2[m.trainIdx].pt);
    }
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    cerr << "Usage: triangulation img1 img2\n";
    return 1;
  }
  Mat first = imread(argv[1]), second = imread(argv[2]);
  if (first.empty() || second.empty()) {
    cerr << "Provide two readable images.\n";
    return 1;
  }
  vector<Point2d> pixels1, pixels2;
  match_features(first, second, pixels1, pixels2);
  if (pixels1.size() < 5) {
    cerr << "Need at least five matching features.\n";
    return 1;
  }

  const Matx33d K(520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  Mat mask, rotation, translation;
  Mat E = findEssentialMat(pixels1, pixels2, K, RANSAC, 0.999, 1.0, mask);
  if (E.rows != 3 || E.cols != 3 ||
      recoverPose(E, pixels1, pixels2, K, rotation, translation, mask) == 0) {
    cerr << "Could not recover a two-view pose.\n";
    return 1;
  }

  // World coordinates coincide with camera 1. Camera 2 sees X as R*X + t.
  const Matx33d R(rotation);
  const Vec3d t(translation);
  const Matx34d P1 = Matx34d::eye();
  Matx34d P2;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) P2(row, col) = R(row, col);
    P2(row, 3) = t[row];
  }
  Mat homogeneous;
  triangulatePoints(K * P1, K * P2, pixels1, pixels2, homogeneous);
  vector<Landmark> points;
  for (int i = 0; i < homogeneous.cols; ++i) {
    if (!mask.at<uchar>(i)) continue;
    const double w = homogeneous.at<double>(3, i);
    if (abs(w) < 1e-10) continue;
    const Vec3d world(homogeneous.at<double>(0, i) / w, homogeneous.at<double>(1, i) / w,
                      homogeneous.at<double>(2, i) / w);
    const Vec3d in_second = R * world + t;
    if (!std::isfinite(norm(world)) || world[2] <= 0 || in_second[2] <= 0) continue;
    points.push_back({world, pixels1[i], pixels2[i], in_second[2]});
    cout << "depth: " << world[2] << '\n';
  }
  if (points.empty()) {
    cerr << "No finite points in front of both cameras.\n";
    return 1;
  }
  cout << "Reconstructed " << points.size() << " points from " << pixels1.size() << " matches.\n";
  TriangulationView view(first, second, points, K, R, t);
  return view.show();
}
