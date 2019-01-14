#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>

struct camera_params {
  float f;
  float cx;
  float cy;

  camera_params() { };

  camera_params(float focal,
    float centerx, float centery) {
    f = focal;
    cx = centerx;
    cy = centery;
  };

  camera_params(const camera_params &obj) {
    f = obj.f;
    cx = obj.cx;
    cy = obj.cy;
  }
};

struct camera_frame_wo_image {
  Eigen::Matrix3f rotation;
  cv::Point3f position;
  camera_params intrinsics;

  camera_frame_wo_image() { };

  camera_frame_wo_image(const camera_frame_wo_image &obj) {
    rotation = obj.rotation;
    position = obj.position;
    intrinsics = obj.intrinsics;
  }

  camera_frame_wo_image(Eigen::Matrix3f rot, cv::Point3f pos, camera_params intr) {
    rotation = rot;
    position = pos;
    intrinsics = intr;
  }

  camera_frame_wo_image(float focal, Eigen::Matrix3f rot, Eigen::Vector3f trans,int cx, int cy) {
    rotation = rot;
    position.x = trans(0, 0);
    position.y = trans(1, 0);
    position.z = trans(2, 0);
    intrinsics = camera_params(focal, cx, cy);
  }
};

struct triangulation_bundle {
  camera_frame_wo_image camera;
  cv::Point2f pt;

  triangulation_bundle() { };

  triangulation_bundle(camera_frame_wo_image cam, 
    cv::Point2f point) {
    camera = camera_frame_wo_image(cam.rotation, cam.position, cam.intrinsics);
    pt = point;
  }
};

/**
 * @brief Traingaulates a set of points
 * @details Takes in points in center subtracted form in triangulation bundle
 * 
 * @param input vector of information for triangulation
 * @return Triangulated 3D point
 */
cv::Point3f Triangulate(std::vector<triangulation_bundle> &input);

#endif
