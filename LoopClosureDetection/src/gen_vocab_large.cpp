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
    cerr << "usage: gen_vocab dataset_directory" << endl;
    return 1;
  }
  string dataset_dir = argv[1];
  ifstream fin(dataset_dir + "/associate.txt");
  if (!fin) {
    cout << "please generate the associate file called associate.txt!" << endl;
    return 1;
  }

  vector<string> rgb_files;
  string rgb_time, rgb_file, depth_time, depth_file;
  while (fin >> rgb_time >> rgb_file >> depth_time >> depth_file) {
    rgb_files.push_back(dataset_dir + "/" + rgb_file);
  }
  if (rgb_files.empty()) return 1;

  cout << "generating features ... " << endl;
  vector<Mat> descriptors;
  Ptr<Feature2D> detector = ORB::create();
  int index = 1;
  size_t feature_count = 0;
  for (string rgb_file : rgb_files) {
    Mat image = imread(rgb_file);
    if (image.empty()) return 1;
    vector<KeyPoint> keypoints;
    Mat descriptor;
    detector->detectAndCompute(image, Mat(), keypoints, descriptor);
    feature_count += descriptor.rows;
    descriptors.push_back(descriptor);
    cout << "extracting features from image " << index++ << endl;
  }
  cout << "extract total " << feature_count << " features." << endl;

  // create vocabulary
  cout << "creating vocabulary, please wait ... " << endl;
  DBoW3::Vocabulary vocab;
  vocab.create(descriptors);
  cout << "vocabulary info: " << vocab << endl;
  vocab.save("vocab_larger.yml.gz");
  cout << "done" << endl;

  return 0;
}