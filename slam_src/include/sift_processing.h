#ifndef SIFT_PROCESSING_H
#define SIFT_PROCESSING_H

#include <opencv2/core/core.hpp>
#include <vector>

void GetCorrespondances(cv::Mat &img1,
  cv::Mat &img2,
  std::vector<cv::Point2f> &old_corr,
  std::vector<cv::Point2f> &new_corr);

#endif
