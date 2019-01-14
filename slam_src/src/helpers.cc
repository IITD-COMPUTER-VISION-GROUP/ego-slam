#include "helpers.h"

void undistort(cv::Mat &img) {
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
  std::vector<double> distCoeffs = {-0.29344778, 0.17523322, 0.00032134, -0.00088967, -0.08528005};
  cv::undistort(temp, img, cameraMatrix, distCoeffs);    
}

bool GetEssentialRT(corr &corres, 
  TwoViewInfo &twoview_info, 
  std::vector<int> &inlier_indices, 
  VerifyTwoViewMatchesOptions& options,
  float focal) {
  CameraIntrinsicsPrior intrinsics1, intrinsics2;
  intrinsics1.focal_length.value = focal;
  intrinsics1.focal_length.is_set = true;
  intrinsics1.principal_point[0].is_set = true;
  intrinsics1.principal_point[0].value = 0.0;
  intrinsics1.principal_point[1].is_set = true;
  intrinsics1.principal_point[1].value = 0.0;
  intrinsics1.aspect_ratio.is_set = true;
  intrinsics1.aspect_ratio.value = 1.0;
  intrinsics1.skew.is_set = true;
  intrinsics1.skew.value = 0.0;
  
  intrinsics2.focal_length.value = focal;
  intrinsics2.focal_length.is_set = true;
  intrinsics2.principal_point[0].is_set = true;
  intrinsics2.principal_point[0].value = 0.0;
  intrinsics2.principal_point[1].is_set = true;
  intrinsics2.principal_point[1].value = 0.0;
  intrinsics2.aspect_ratio.is_set = true;
  intrinsics2.aspect_ratio.value = 1.0;
  intrinsics2.skew.is_set = true;
  intrinsics2.skew.value = 0.0;

  std::vector<FeatureCorrespondence> correspondences;
  for (int i=0; i<corres.p1.size(); i++) {
    FeatureCorrespondence tmp;
    tmp.feature1.x() = corres.p1[i].x;
    tmp.feature1.y() = corres.p1[i].y;
    tmp.feature2.x() = corres.p2[i].x;
    tmp.feature2.y() = corres.p2[i].y;
    correspondences.push_back(tmp);
  }
  return VerifyTwoViewMatches(options, 
      intrinsics1,
      intrinsics2, 
      correspondences, 
      &twoview_info, 
      &inlier_indices);
}

bool VerifyTwoViewMatches(
  const VerifyTwoViewMatchesOptions& options,
  const CameraIntrinsicsPrior& intrinsics1,
  const CameraIntrinsicsPrior& intrinsics2,
  const std::vector<FeatureCorrespondence>& correspondences,
  TwoViewInfo* twoview_info,
  std::vector<int>* inlier_indices) {
  if (correspondences.size() < options.min_num_inlier_matches) {
    return false;
  }

  // Estimate the two view info. If we fail to estimate a two view info then do
  // not add this view pair to the verified matches.
  if (!EstimateTwoViewInfo(options.estimate_twoview_info_options,
    intrinsics1,
    intrinsics2,
    correspondences,
    twoview_info,
    inlier_indices)) {
    return false;
  }

  // If there were not enough inliers, return false and do not bother to
  // (potentially) run bundle adjustment.
  if (inlier_indices->size() < options.min_num_inlier_matches) {
    return false;
  }
  return true;
}

void ShowCorres(std::string FLAGS_dirname, corr compressed) {
  cv::Mat im1 = cv::imread(FLAGS_dirname+"/img_"+std::to_string(compressed.frame_1)+".jpg");
  cv::Mat im2 = cv::imread(FLAGS_dirname+"/img_"+std::to_string(compressed.frame_2)+".jpg");
  cv::Size sz1 = im1.size();
  cv::resize(im1, im1, cv::Size(sz1.width/2, sz1.height/2));
  cv::resize(im2, im2, cv::Size(sz1.width/2, sz1.height/2));
  sz1 = im1.size();
  system(("mkdir " + FLAGS_dirname + "/corr_" + std::to_string(compressed.frame_1)+"_"+std::to_string(compressed.frame_2)).c_str());
  for (int i=0; i<compressed.p1.size()/10;i++) {
    cv::Mat im3(sz1.height, 2*sz1.width, CV_8UC3);
    cv::Mat left(im3, cv::Rect(0,0,sz1.width, sz1.height));
    im1.copyTo(left);
    cv::Mat right(im3, cv::Rect(sz1.width,0,sz1.width, sz1.height));
    im2.copyTo(right);
    for (int j=10*i; j<10*(i+1); j++) {
      cv::circle(im3, cv::Point2f(compressed.p1[j].x/2,compressed.p1[j].y/2),3, cv::Scalar(255,0,0), -1);
      cv::circle(im3, cv::Point2f(compressed.p2[j].x/2+sz1.width,compressed.p2[j].y/2),3, cv::Scalar(255,0,0), -1);
      cv::line(im3, cv::Point2f(compressed.p1[j].x/2,compressed.p1[j].y/2), cv::Point2f(compressed.p2[j].x/2+sz1.width,compressed.p2[j].y/2), cv::Scalar(0,0,255));
    }
    cv::imwrite(FLAGS_dirname+"/corr_"+std::to_string(compressed.frame_1)+"_"+std::to_string(compressed.frame_2)+"/"+std::to_string(i)+".jpg", im3);
  }
}

