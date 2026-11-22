#pragma once

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "image_panel.hpp"

namespace flow {

// The renderer may draw on either frame. Video playback keeps clean decoder frames.
using Renderer = std::function<cv::Mat(cv::Mat, cv::Mat, bool)>;

inline bool same_file(const std::string &input, const std::string &output) {
  struct stat a, b;
  return stat(input.c_str(), &a) == 0 && stat(output.c_str(), &b) == 0 &&
         a.st_dev == b.st_dev && a.st_ino == b.st_ino;
}

inline int play_video(const std::string &input, const std::string &output,
                      const std::string &title, const Renderer &render) {
  if (same_file(input, output)) {
    std::cerr << "Choose an output file different from the input video.\n";
    return 1;
  }
  cv::VideoCapture capture(input);
  cv::Mat previous, current;
  if (!capture.isOpened() || !capture.read(previous) || !capture.read(current)) {
    std::cerr << "Expected a readable video with at least two frames: " << input << '\n';
    return 1;
  }
  const double source_fps = capture.get(cv::CAP_PROP_FPS);
  const double fps = std::isfinite(source_fps) && source_fps > 0 ? source_fps : 30.0;
  const bool display = !std::getenv("SLAM_HEADLESS");
  cv::VideoWriter writer;
  int pairs = 0;
  do {
    const auto start = std::chrono::steady_clock::now();
    if (current.size() != previous.size()) {
      std::cerr << "Frame size changed during playback.\n";
      return 1;
    }
    cv::Mat canvas = render(previous.clone(), current.clone(), false);
    if (canvas.empty()) return 1;
    ++pairs;
    cv::putText(canvas, "Frames " + std::to_string(pairs) + " -> " + std::to_string(pairs + 1),
                {canvas.cols - 190, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    if (!output.empty()) {
      if (pairs == 1 && !writer.open(output, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                     fps, canvas.size())) {
        std::cerr << "Could not open output video: " << output << '\n';
        return 1;
      }
      writer.write(canvas);
    }
    if (display) {
      if (pairs == 1) {
        cv::namedWindow(title, cv::WINDOW_NORMAL);
        cv::resizeWindow(title, canvas.cols, canvas.rows);
      }
      cv::imshow(title, canvas);
      const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
      const int key = cv::waitKey(std::max(1, cvRound(1000.0 / fps - elapsed.count())));
      if (key == 27 || key == 'q' || cv::getWindowProperty(title, cv::WND_PROP_VISIBLE) == 0) break;
    }
    std::swap(previous, current);
  } while (capture.read(current));
  std::cout << "Processed " << pairs << " frame pairs (" << fps << " fps).\n";
  return 0;
}

inline int run(int argc, char **argv, const std::string &title, const std::string &png,
               const Renderer &render) {
  try {
    if ((argc == 3 || argc == 5) && std::string(argv[1]) == "--video" &&
        (argc == 3 || std::string(argv[3]) == "--output")) {
      return play_video(argv[2], argc == 5 ? argv[4] : "", title, render);
    }
    if (argc != 3 || std::string(argv[1]) == "--video") {
      std::cerr << "Usage: " << argv[0] << " first_image second_image\n"
                << "       " << argv[0] << " --video input_video [--output result.mp4]\n";
      return 1;
    }
    cv::Mat first = cv::imread(argv[1]), second = cv::imread(argv[2]);
    if (first.empty() || second.empty() || first.size() != second.size()) {
      std::cerr << "Provide two readable images of the same size.\n";
      return 1;
    }
    cv::Mat canvas = render(first, second, true);
    if (canvas.empty()) return 1;
    if (std::getenv("SLAM_HEADLESS")) return cv::imwrite(png, canvas) ? 0 : 1;
    lecture::show_result(title, canvas);
    return 0;
  } catch (const cv::Exception &error) {
    std::cerr << title << ": " << error.what() << '\n';
    return 1;
  }
}

} // namespace flow
