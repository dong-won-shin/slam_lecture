#include <iomanip>
#include <iostream>
#include <vector>

#include <opencv2/video/tracking.hpp>
#if CV_VERSION_MAJOR < 4
#include <opencv2/optflow.hpp>
#endif

#include "flow_stats.hpp"
#include "flow_example.hpp"

static cv::Mat render_dis(cv::Mat first, cv::Mat second, bool report,
                          const cv::Ptr<cv::DenseOpticalFlow> &dis) {
  cv::Mat first_gray, second_gray;
  cv::cvtColor(first, first_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(second, second_gray, cv::COLOR_BGR2GRAY);

  // At pixel (x, y) in frame 1, the flow stores its (dx, dy) into frame 2.
  cv::Mat field;
  dis->calc(first_gray, second_gray, field);
  if (!cv::checkRange(field)) {
    std::cerr << "DIS produced non-finite flow vectors.\n";
    return {};
  }

  cv::Mat components[2], magnitude, angle;
  cv::split(field, components);
  cv::cartToPolar(components[0], components[1], magnitude, angle, true);
  // Hue encodes direction. A fixed brightness scale makes image pairs comparable.
  cv::Mat hsv_channels[3];
  angle.convertTo(hsv_channels[0], CV_8U, 0.5);
  hsv_channels[1] = cv::Mat(field.size(), CV_8U, cv::Scalar(255));
  magnitude.convertTo(hsv_channels[2], CV_8U, 255.0 / 32.0);
  cv::Mat hsv, colors;
  cv::merge(hsv_channels, 3, hsv);
  cv::cvtColor(hsv, colors, cv::COLOR_HSV2BGR);

  const int spacing = 24;
  for (int y = spacing / 2; y < field.rows; y += spacing) {
    for (int x = spacing / 2; x < field.cols; x += spacing) {
      const cv::Point2f origin(x, y);
      const cv::Point2f motion = field.at<cv::Point2f>(y, x);
      cv::arrowedLine(first, origin, origin + motion, cv::Scalar(40, 220, 255),
                      1, cv::LINE_AA, 0, 0.25);
    }
  }

  if (report) {
    const std::vector<float> dx(components[0].begin<float>(), components[0].end<float>());
    const std::vector<float> dy(components[1].begin<float>(), components[1].end<float>());
    std::cout << "Dense vectors: " << field.total() << std::fixed << std::setprecision(2)
              << "\nMedian flow (dx, dy) in pixels: "
              << flow::median(dx) << ", "
              << flow::median(dy) << '\n';
  }

  cv::Mat overview(458, 1040, CV_8UC3, cv::Scalar(29, 26, 24));
  lecture::draw_panel(overview, {12, 8, 502, 407}, first, "Previous / sampled flow vectors");
  lecture::draw_panel(overview, {526, 8, 502, 407}, colors, "DIS / dense flow");
  cv::Mat legend_hsv(1, 4, CV_8UC3);
  const char *directions[] = {"Right", "Down", "Left", "Up"};
  for (int i = 0; i < 4; ++i)
    legend_hsv.at<cv::Vec3b>(0, i) = cv::Vec3b(i * 45, 255, 255);
  cv::Mat legend;
  cv::cvtColor(legend_hsv, legend, cv::COLOR_HSV2BGR);
  const cv::Scalar text(235, 235, 235);
  for (int i = 0; i < 4; ++i) {
    const int x = 12 + 90 * i;
    cv::rectangle(overview, {x, 431, 13, 13}, cv::Scalar(legend.at<cv::Vec3b>(0, i)), cv::FILLED);
    cv::putText(overview, directions[i], {x + 19, 443}, cv::FONT_HERSHEY_SIMPLEX, 0.5, text, 1, cv::LINE_AA);
  }
  cv::putText(overview, "Brightness: 0 to 32 px", {526, 443}, cv::FONT_HERSHEY_SIMPLEX,
              0.5, text, 1, cv::LINE_AA);

  return overview;
}

int main(int argc, char **argv) {
#if CV_VERSION_MAJOR >= 4
  auto dis = cv::DISOpticalFlow::create(cv::DISOpticalFlow::PRESET_MEDIUM);
#else
  auto dis = cv::optflow::createOptFlow_DIS(cv::optflow::DISOpticalFlow::PRESET_MEDIUM);
#endif
  return flow::run(argc, argv, "DIS optical flow", "dis_optical_flow.png",
                   [&dis](cv::Mat first, cv::Mat second, bool report) {
                     return render_dis(first, second, report, dis);
                   });
}
