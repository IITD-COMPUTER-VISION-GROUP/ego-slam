#ifndef CERES_FIT_H
#define CERES_FIT_H

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "pointcloud.h"

struct PlanarResidual {
  PlanarResidual(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  const T* const d, T* residual) const {
    residual[0] = d[0] - (x_ * a[0] + y_ * b[0] + z_ * c[0]);
    return true;
  }

 private:
  const double x_;
  const double y_;
  const double z_;
};

struct PerpResidual {
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, const T* const x3,
                  const T* const x4, const T* const x5, const T* const x6,
                  T* residual) const {
    residual[0] = 100.0 * (x1[0] * x4[0] + x2[0] * x5[0] + x3[0] * x6[0]);
    return true;
  }
};

struct PerpTrajectoryResidual {
  PerpTrajectoryResidual(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, const T* const x3,
                  T* residual) const {
    residual[0] = 100.0 * (x1[0] * x_ + x2[0] * y_ + x3[0] * z_);
    return true;
  }

 private:
  const double x_;
  const double y_;
  const double z_;
};

void optimize(int type, plane& rf, plane& lft, plane& rt,
              std::vector<cv::Point3f>& pts_rf,
              std::vector<cv::Point3f>& pts_left,
              std::vector<cv::Point3f>& pts_right,
              std::vector<cv::Point3f>& trajectory, float mindistance);

#endif
