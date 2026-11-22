#include <iostream>
#include "loop_closure_view.hpp"

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: loop_closure vocabulary image_directory\n";
    return 1;
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

  std::vector<LoopFrame> frames(10);
  std::vector<DBoW3::BowVector> words(frames.size());
  DBoW3::Database db(vocab, false, 0);
  auto orb = cv::ORB::create();
  for (size_t i = 0; i < frames.size(); ++i) {
    auto &frame = frames[i];
    const std::string path = std::string(argv[2]) + "/" + std::to_string(i + 1) + ".png";
    frame.image = cv::imread(path);
    if (frame.image.empty()) {
      std::cerr << "Could not read " << path << '\n';
      return 1;
    }
    orb->detectAndCompute(frame.image, cv::noArray(), frame.keypoints, frame.descriptors);
    vocab.transform(frame.descriptors, words[i]);
    // Query before inserting this frame, as in an online place-recognition system.
    if (i > 0) db.query(words[i], frame.candidates, 3);
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
    for (const auto &r : frames[i].candidates)
      std::cout << "  " << r.Id + 1 << ".png (" << r.Score << ")";
    std::cout << '\n';
  }
  LoopClosureView view(frames);
  return view.show();
}
