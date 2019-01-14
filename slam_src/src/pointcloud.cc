#include "pointcloud.h"
#include <unordered_set>

void segment_Points(std::vector<cv::Point3f> &inputpoints,
                    std::vector<int> &in_inliers, plane &p, float distance, bool side) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Fill in the cloud data
  cloud->width = inputpoints.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = inputpoints[i].x;
    cloud->points[i].y = inputpoints[i].y;
    cloud->points[i].z = inputpoints[i].z;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  if (side) {
    std::cout << "Entered\n";
    Eigen::Vector3f axis;
    axis.setZero();
    axis(0,0) = -1.0;
    seg.setAxis(axis);
    seg.setEpsAngle(10.0f*(M_PI)/180.0);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  } else {
    seg.setModelType(pcl::SACMODEL_PLANE);
  }
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  p.a = coefficients->values[0];
  p.b = coefficients->values[1];
  p.c = coefficients->values[2];
  p.d = coefficients->values[3];

  in_inliers.resize(0);
  for (int i = 0; i < inliers->indices.size(); i++) {
    in_inliers.push_back(inliers->indices[i]);
  }
}

std::vector<cv::Point3f> filterPoints(std::vector<cv::Point3f> &input,
                                      std::vector<int> &inliers,
                                      std::vector<cv::Point3f> &valid) {
  std::vector<cv::Point3f> answer;
  valid.clear();
  std::unordered_set<int> frames;
  for (auto it : inliers) {
    valid.push_back(input[it]);
    frames.insert(it);
  }
  for (int i = 0; i < input.size(); i++) {
    if (frames.find(i) == frames.end()) {
      answer.push_back(input[i]);
    }
  }
  return answer;
}

void fitPlane(std::vector<cv::Point3f> &inpoints,
              std::vector<cv::Point3f> &planepts, plane &p, float dist, bool side) {
  std::vector<int> inliers1;
  segment_Points(inpoints, inliers1, p, dist, side);
  std::vector<cv::Point3f> remaining_pts =
      filterPoints(inpoints, inliers1, planepts);
  inpoints = remaining_pts;
}

void fit3Planes(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3, plane &p1, plane &p2,
                plane &p3, float distance) {
  std::vector<int> inliers1, inliers2, inliers3;
  segment_Points(inputpoints, inliers1, p1, distance);
  std::vector<cv::Point3f> remaining_pts =
      filterPoints(inputpoints, inliers1, plane1);

  segment_Points(remaining_pts, inliers2, p2, distance);
  std::vector<cv::Point3f> remaining_pts_1 =
      filterPoints(remaining_pts, inliers2, plane2);

  segment_Points(remaining_pts_1, inliers3, p3, distance);
  std::vector<cv::Point3f> remaining_pts2 =
      filterPoints(remaining_pts_1, inliers3, plane3);
}
