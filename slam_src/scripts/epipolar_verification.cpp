#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <gflags/gflags.h>

DEFINE_string(dirname, "epipolar/temp", "Directory to read from");

float focal = 1690; 

cv::Point2f center(640, 360);

struct file_props {
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  std::string file_path;

  file_props() { };

  file_props(Eigen::Matrix3f r, Eigen::Vector3f t, std::string pth) {
    rotation = r;
    translation = t;
    file_path = pth;
  }
};

struct Correspondance {
  std::unordered_map<int, file_props> files;
  std::unordered_map<int, std::unordered_map<int, cv::Point2f> > all_corr;
};

Correspondance load_data_from_disk(std::string dirname) {
  Correspondance answer;  
  // Read list_focal.txt
  FILE *listfocal = fopen((dirname + "/../list_focal.txt").c_str(), "r");
  int line_number = 0;
  std::string filename;
  char fname[256];
  std::vector<std::string> filenames;
  std::cout << "Reading now\n";
  while (fscanf(listfocal, "%s %f\n", fname, &focal) != EOF) {
    filenames.push_back(fname);
    line_number++;
  }
  assert(line_number == filenames.size());
  // for (int i=0; i<filenames.size(); i++) {
  //   std::cout << i << "\t" << filenames[i] << "\n";
  // }

  std::vector<int> clusternames;
  // Read cluster_list_map.txt
  int temp;
  FILE *cluster_list = fopen((dirname+"/cluster_list_map.txt").c_str(),"r");
  while (fscanf(cluster_list, "%d\n", &temp) != EOF) {
    clusternames.push_back(temp);
  }

  // Read RTglobal.txt

  FILE *rtglobal = fopen((dirname+"/RTglobal.txt").c_str(), "r");
  int counter = 0;
  Eigen::Matrix3f r1;
  Eigen::Vector3f t1;
  std::vector<Eigen::Matrix3f> all_rotations;
  std::vector<Eigen::Vector3f> all_translations;
  while (fscanf(rtglobal, "%f %f %f %f %f %f %f %f %f\n", 
      &r1.data()[0], 
      &r1.data()[3], 
      &r1.data()[6], 
      &r1.data()[1], 
      &r1.data()[4], 
      &r1.data()[7], 
      &r1.data()[2], 
      &r1.data()[5], 
      &r1.data()[8]) != EOF) {
    fscanf(rtglobal, "%f %f %f\n", &t1.data()[0], &t1.data()[1], &t1.data()[2]);
    all_rotations.push_back(r1);
    all_translations.push_back(t1);
  }

  for (int i=0; i<clusternames.size(); i++) {
    answer.files[i] = file_props(all_rotations[clusternames[i]], all_translations[clusternames[i]], filenames[clusternames[i]]);
  }

  // Read matches
  FILE *matches = fopen((dirname+"/matches_forRtinlier5point.txt").c_str(), "r");
  int numpairs;
  fscanf(matches, "%d\n", &numpairs);
  for (int i=0; i<numpairs; i++) {
    int f1, f2, num_cors;
    fscanf(matches, "%d %d %d\n", &f1, &f2, &num_cors);
    for (int j=0; j<num_cors; j++) {
      int sft1, sft2;
      float p1x, p2x, p1y, p2y;
      fscanf(matches, "%d %f %f %d %f %f\n", 
        &sft1, &p1x, &p1y, &sft2, &p2x, &p2y);
      assert(sft1 == sft2);
      
      answer.all_corr[sft1][f1] = cv::Point2f(p1x, p1y);
      answer.all_corr[sft2][f2] = cv::Point2f(p2x, p2y);
    }
  }
  return answer;
}

void ComputeEpipoles(float focal, 
  Eigen::Matrix3d Rotation, 
  Eigen::Vector3d translation,
  cv::Point2f &epi0,
  cv::Point2f &epi1) {
  Eigen::Vector3d e = Rotation.transpose() * translation;
  epi0.x = focal*e.data()[0]/e.data()[2];
  epi0.y = focal*e.data()[1]/e.data()[2];

  epi1.x = focal*translation.data()[0]/translation.data()[2];
  epi1.y = focal*translation.data()[1]/translation.data()[2];
}

bool ValidPoints(std::unordered_map<int, file_props> &files,
  std::unordered_map<int, cv::Point2f> corres,
  std::string data_dir) {
  // Find all epipolar lines between 
  for (auto it: corres) {
    assert (files.find(it.first)!=files.end());
    std::cerr << data_dir + "/../" + files[it.first].file_path << "\n"; 
    cv::Mat img = cv::imread(data_dir + "/../" + files[it.first].file_path);
    cv::namedWindow("img_" + std::to_string(it.first));
    cv::Mat temp = img.clone();
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1134;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 645;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 1126;
    cameraMatrix.at<double>(1, 2) = 364;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;
    std::vector<float> distCoeffs = {-0.29344778, 0.17523322, 0.00032134, -0.00088967, -0.08528005};
    cv::undistort(temp, img, cameraMatrix, distCoeffs);
    cv::circle(img, it.second + center, 5, cv::Scalar(255, 0, 0), -1);
    cv::imshow("img_" + std::to_string(it.first), img);
  }
  if (cv::waitKey(0) == 27)
    return true;
  return true;
}

int main(int argc, char **argv)
{
  google::SetUsageMessage("slam --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  Correspondance loaded = load_data_from_disk(FLAGS_dirname);

  std::cout << "Size of files: " << loaded.files.size() << "\n";
  std::cout << "Size of matches: " << loaded.all_corr.size() << "\n";


  int min_num = 1000000;
  int max_num = 0;
  for (auto it: loaded.all_corr) {
    min_num = std::min(min_num, (int) it.second.size());
    if (it.second.size() > max_num) {
      max_num = it.second.size();
      std::cout << it.first << "\t" << it.second.size() << "\n";
    }
  }
  std::cout << "min " <<  min_num << " max " << max_num << "\n";
  ValidPoints(loaded.files, loaded.all_corr[0], FLAGS_dirname);
  return 0;
}
