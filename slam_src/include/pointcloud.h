#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/core/core.hpp>
#include <vector>

struct plane {
  float a, b, c, d;

  plane(){};
  plane(float x1, float x2, float x3, float x4) : a(x1), b(x2), c(x3), d(x4){};

  float norm() { return sqrt(a * a + b * b + c * c); }

  float dotp(plane p) {
    float answer = a * p.a + b * p.b + c * p.c;
    float denom = norm() * p.norm();
    if (denom == 0) {
      return 0;
    } else {
      return answer / denom;
    }
  }

  Eigen::Vector3f GetNormal() {
    Eigen::Vector3f answer;
    answer(0, 0) = a;
    answer(1, 0) = b;
    answer(2, 0) = c;
    return answer;
  }

  void rotate(Eigen::Matrix3f rot) {
    Eigen::Vector3f temp;
    temp(0, 0) = a;
    temp(1, 0) = b;
    temp(2, 0) = c;
    temp = (temp.transpose() * rot.transpose()).transpose();
    a = temp(0, 0);
    b = temp(1, 0);
    c = temp(2, 0);
  }

  void normalize() {
    float n = norm();
    a /= n;
    b /= n;
    c /= n;
    d /= n;
  }

  float value(cv::Point3f p) { return a * p.x + b * p.y + c * p.z + d; }

  float distance(cv::Point3f p) { return fabs(value(p)) / norm(); }

  void shift(cv::Point3f p) { d = d - (a * p.x + b * p.y + c * p.z); }

  friend std::ostream &operator<<(std::ostream &stream, plane &p) {
    stream << "(" << p.a << ", " << p.b << ", " << p.c << ", " << p.d << ")";
    return stream;
  }
};

void segment_Points(std::vector<cv::Point3f> &inputpoints,
                    std::vector<int> &inliers, plane &p, float distance, bool side=false);

void fitPlane(std::vector<cv::Point3f> &inpoints,
              std::vector<cv::Point3f> &planepts, plane &p, float dist, bool side=false);

void fit3Planes(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3, plane &p1, plane &p2,
                plane &p3, float distance);

#endif
