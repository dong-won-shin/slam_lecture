#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgcodecs.hpp>

// Both examples read the same chronological sequence, named 1.png through N.png.
inline bool load_numbered_images(const std::string &directory, std::vector<cv::Mat> &images) {
  std::vector<cv::String> files;
  try {
    cv::glob(directory + "/*.png", files);
  } catch (const cv::Exception &error) {
    std::cerr << error.what() << '\n';
    return false;
  }

  size_t count = 0;
  for (const auto &file : files) {
    const std::string name = file.substr(file.find_last_of("/\\") + 1);
    const auto stem = name.substr(0, name.size() - 4);
    if (!stem.empty() && stem.find_first_not_of("0123456789") == std::string::npos) ++count;
  }
  if (count == 0) {
    std::cerr << "No numbered PNG images in " << directory << '\n';
    return false;
  }
  for (size_t i = 1; i <= count; ++i) {
    const auto path = directory + "/" + std::to_string(i) + ".png";
    auto image = cv::imread(path);
    if (image.empty()) {
      std::cerr << "Could not read " << path << '\n';
      return false;
    }
    images.push_back(image);
  }
  return true;
}
