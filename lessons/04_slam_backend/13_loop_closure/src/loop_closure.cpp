#include <iostream>
#include <opencv2/calib3d.hpp>
#include "loop_closure_view.hpp"
#include "numbered_images.hpp"

static LoopCandidate verify_candidate(const LoopFrame &query, const LoopFrame &past,
                                      const DBoW3::Result &retrieval) {
  LoopCandidate result{retrieval, {}, {}};
  if (query.descriptors.empty() || past.descriptors.rows < 2) return result;

  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> forward;
  std::vector<cv::DMatch> reverse;
  matcher.knnMatch(query.descriptors, past.descriptors, forward, 2);
  matcher.match(past.descriptors, query.descriptors, reverse);
  for (const auto &pair : forward) {
    if (pair.size() != 2) continue;
    const auto &m = pair.front();
    if (m.distance < 0.75f * pair[1].distance && reverse[m.trainIdx].trainIdx == m.queryIdx)
      result.matches.push_back(m);
  }
  if (result.matches.size() < LoopCandidate::min_inliers) return result;

  std::vector<cv::Point2f> from, to;
  for (const auto &m : result.matches) {
    from.push_back(query.keypoints[m.queryIdx].pt);
    to.push_back(past.keypoints[m.trainIdx].pt);
  }
  std::vector<uchar> mask;
  const cv::Mat F = cv::findFundamentalMat(from, to, cv::FM_RANSAC, 1.5, 0.99, mask);
  if (F.rows == 3 && F.cols == 3 && cv::checkRange(F)) {
    for (size_t i = 0; i < mask.size(); ++i)
      if (mask[i]) result.inliers.push_back(result.matches[i]);
    std::sort(result.inliers.begin(), result.inliers.end());
  }
  return result;
}

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "Usage: loop_closure vocabulary image_directory [query_number]\n";
    return 1;
  }
  std::vector<cv::Mat> images;
  if (!load_numbered_images(argv[2], images)) return 1;
  std::vector<LoopFrame> frames(images.size());
  size_t initial_query = frames.size() - 1;
  if (argc == 4) {
    char *end = nullptr;
    const long number = std::strtol(argv[3], &end, 10);
    if (*end || number < 1 || number > static_cast<long>(frames.size())) {
      std::cerr << "Query number must be between 1 and " << frames.size() << '\n';
      return 1;
    }
    initial_query = static_cast<size_t>(number - 1);
  }
  DBoW3::Vocabulary vocab;
  try {
    vocab.load(argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  } catch (const std::string &error) {
    std::cerr << error << '\n';
    return 1;
  }
  if (vocab.empty()) {
    std::cerr << "Empty vocabulary.\n";
    return 1;
  }

  std::vector<DBoW3::BowVector> words(frames.size());
  DBoW3::Database db(vocab, false, 0);
  auto orb = cv::ORB::create();
  for (size_t i = 0; i < frames.size(); ++i) {
    auto &frame = frames[i];
    frame.image = images[i];
    orb->detectAndCompute(frame.image, cv::noArray(), frame.keypoints, frame.descriptors);
    vocab.transform(frame.descriptors, words[i]);
    // Query before inserting this frame, as in an online place-recognition system.
    DBoW3::QueryResults retrieved;
    if (i > 0) db.query(words[i], retrieved, 3);
    for (const auto &r : retrieved)
      frame.candidates.push_back(verify_candidate(frame, frames[r.Id], r));
    db.add(words[i]);
  }

  // Keep the original all-pairs comparison and full-database search for the lecture.
  std::cout << "comparing images with images (zero-based database IDs)\n";
  for (size_t i = 0; i < frames.size(); ++i)
    for (size_t j = i; j < frames.size(); ++j)
      std::cout << "image " << i << " vs image " << j << " : " << vocab.score(words[i], words[j]) << '\n';
  std::cout << "database info: " << db << '\n';
  for (size_t i = 0; i < frames.size(); ++i) {
    DBoW3::QueryResults full;
    db.query(words[i], full, 4);
    std::cout << "searching for image " << i << " returns " << full << '\n';
    std::cout << "Earlier candidates for " << i + 1 << ".png:";
    for (const auto &candidate : frames[i].candidates) {
      const auto &r = candidate.retrieval;
      std::cout << "  " << r.Id + 1 << ".png (" << r.Score << ", " << candidate.matches.size() << " matches, ";
      if (candidate.matches.size() >= LoopCandidate::min_inliers)
        std::cout << candidate.inliers.size() << " inliers, ";
      std::cout << (candidate.verified() ? "verified" : "rejected") << ")";
    }
    const int best = frames[i].best_verified_rank();
    std::cout << "\nVerified loop for " << i + 1 << ".png: ";
    if (best >= 0) std::cout << frames[i].candidates[best].retrieval.Id + 1 << ".png";
    else std::cout << "none";
    std::cout << '\n';
  }
  LoopClosureView view(frames, initial_query);
  return view.show();
}
