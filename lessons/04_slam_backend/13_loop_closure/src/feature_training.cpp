#include "numbered_images.hpp"
#include "DBoW3/DBoW3.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
  if (argc != 2) {
    cerr << "Usage: feature_training image_directory\n";
    return 1;
  }
  cout << "reading images... " << endl;
  vector<Mat> images;
  if (!load_numbered_images(argv[1], images)) return 1;
  // detect ORB features
  cout << "detecting ORB features ... " << endl;
  Ptr<Feature2D> detector = ORB::create();
  vector<Mat> descriptors;
  for (Mat &image : images) {
    vector<KeyPoint> keypoints;
    Mat descriptor;
    detector->detectAndCompute(image, Mat(), keypoints, descriptor);
    descriptors.push_back(descriptor);
  }

  // create vocabulary
  cout << "creating vocabulary ... " << endl;
  DBoW3::Vocabulary vocab;
  vocab.create(descriptors);
  cout << "vocabulary info: " << vocab << endl;
  vocab.save("vocabulary.yml.gz");
  cout << "done" << endl;

  return 0;
}