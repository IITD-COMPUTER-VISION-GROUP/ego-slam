// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "estimate_twoview_info.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <vector>

#include "sample_consensus_estimator.h"
#include "camera_intrinsics_prior.h"
#include "estimate_relative_pose.h"
#include "estimate_uncalibrated_relative_pose.h"
#include "feature_correspondence.h"
#include "pose_util.h"
#include "triangulation.h"
#include "twoview_info.h"
#include "sfm_types.h"

//namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

struct CameraIntrinsics {
  double focal_length = 1.0;
  double principal_point[2];
  double aspect_ratio = 1.0;
  double skew = 0.0;
};

void SetCameraIntrinsics(const CameraIntrinsicsPrior& prior,
                         CameraIntrinsics* intrinsics) {
  if (prior.focal_length.is_set) {
    intrinsics->focal_length = prior.focal_length.value;
  }

  if (prior.principal_point[0].is_set && prior.principal_point[1].is_set) {
    intrinsics->principal_point[0] = prior.principal_point[0].value;
    intrinsics->principal_point[1] = prior.principal_point[1].value;
  } else {
    intrinsics->principal_point[0] = prior.image_width / 2.0;
    intrinsics->principal_point[1] = prior.image_height / 2.0;
  }

  if (prior.aspect_ratio.is_set) {
    intrinsics->aspect_ratio = prior.aspect_ratio.value;
  }

  if (prior.skew.is_set) {
    intrinsics->skew = prior.skew.value;
  }
}

void NormalizeFeature(const CameraIntrinsics& intrinsics, Vector2d* feature) {
  feature->y() = (feature->y() - intrinsics.principal_point[1]) /
                 (intrinsics.focal_length * intrinsics.aspect_ratio);
  feature->x() = (feature->x() - intrinsics.skew * feature->y() -
                  intrinsics.principal_point[0]) / intrinsics.focal_length;
}

// Normalizes the image features by the camera intrinsics.
void NormalizeFeatures(
    const CameraIntrinsicsPrior& prior1,
    const CameraIntrinsicsPrior& prior2,
    const std::vector<FeatureCorrespondence>& correspondences,
    std::vector<FeatureCorrespondence>* normalized_correspondences) {
  *CHECK_NOTNULL(normalized_correspondences) = correspondences;

  CameraIntrinsics intrinsics1, intrinsics2;
  SetCameraIntrinsics(prior1, &intrinsics1);
  SetCameraIntrinsics(prior2, &intrinsics2);
  for (int i = 0; i < correspondences.size(); i++) {
    NormalizeFeature(intrinsics1, &normalized_correspondences->at(i).feature1);
    NormalizeFeature(intrinsics2, &normalized_correspondences->at(i).feature2);
  }
}

bool EstimateTwoViewInfoCalibrated(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  // Normalize features w.r.t focal length.
  std::vector<FeatureCorrespondence> normalized_correspondences;
  NormalizeFeatures(intrinsics1,
                    intrinsics2,
                    correspondences,
                    &normalized_correspondences);

  // Set the ransac parameters.
  RansacParameters ransac_options;
  ransac_options.failure_probability = 1.0 - options.expected_ransac_confidence;
  ransac_options.min_iterations = options.min_ransac_iterations;
  ransac_options.max_iterations = options.max_ransac_iterations;
  ransac_options.error_thresh =
      options.max_sampson_error_pixels * options.max_sampson_error_pixels /
      (intrinsics1.focal_length.value * intrinsics2.focal_length.value);
  ransac_options.use_mle = options.use_mle;

  RelativePose relative_pose;
  RansacSummary summary;
  if (!EstimateRelativePose(ransac_options,
                            options.ransac_type,
                            normalized_correspondences,
                            &relative_pose,
                            &summary)) {
    return false;
  }

  AngleAxisd rotation(relative_pose.rotation);

  // Set the twoview info.
  twoview_info->rotation_2 = rotation.angle() * rotation.axis();
  twoview_info->rotationmat_2 = relative_pose.rotation;
  twoview_info->translation_2 = -relative_pose.rotation * relative_pose.position;
  twoview_info->position_2 = relative_pose.position;
  twoview_info->focal_length_1 = intrinsics1.focal_length.value;
  twoview_info->focal_length_2 = intrinsics2.focal_length.value;
  twoview_info->num_verified_matches = summary.inliers.size();
  twoview_info->essential_mat = relative_pose.essential_matrix;
  *inlier_indices = summary.inliers;
  twoview_info->cost = summary.mle;
  return true;
}

bool EstimateTwoViewInfoUncalibrated(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  // Normalize features w.r.t principal point.
  std::vector<FeatureCorrespondence> centered_correspondences;
  NormalizeFeatures(intrinsics1,
                    intrinsics2,
                    correspondences,
                    &centered_correspondences);

  // Set the ransac parameters.
  RansacParameters ransac_options;
  ransac_options.failure_probability = 1.0 - options.expected_ransac_confidence;
  ransac_options.min_iterations = options.min_ransac_iterations;
  ransac_options.max_iterations = options.max_ransac_iterations;
  ransac_options.error_thresh =
      options.max_sampson_error_pixels * options.max_sampson_error_pixels;

  UncalibratedRelativePose relative_pose;
  RansacSummary summary;
  if (!EstimateUncalibratedRelativePose(ransac_options,
                                        options.ransac_type,
                                        centered_correspondences,
                                        &relative_pose,
                                        &summary)) {
    return false;
  }

  AngleAxisd rotation(relative_pose.rotation);

  // Set the twoview info.
  twoview_info->rotation_2 = rotation.angle() * rotation.axis();
  twoview_info->position_2 = relative_pose.position;
  twoview_info->focal_length_1 = relative_pose.focal_length1;
  twoview_info->focal_length_2 = relative_pose.focal_length2;

  // Get the number of verified features.
  twoview_info->num_verified_matches = summary.inliers.size();
  *inlier_indices = summary.inliers;

  return true;
}

}  // namespace

bool EstimateTwoViewInfo(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices) {
  CHECK_NOTNULL(twoview_info);
  CHECK_NOTNULL(inlier_indices)->clear();

  // Case where both views are calibrated.
  if (intrinsics1.focal_length.is_set && intrinsics2.focal_length.is_set) {
    return EstimateTwoViewInfoCalibrated(options,
                                         intrinsics1,
                                         intrinsics2,
                                         correspondences,
                                         twoview_info,
                                         inlier_indices);
  }

  // Only one of the focal lengths is set.
  if (intrinsics1.focal_length.is_set || intrinsics2.focal_length.is_set) {
    LOG(WARNING) << "Solving for two view infos when exactly one view is "
                    "calibrated has not been implemented yet. Treating both "
                    "views as uncalibrated instead.";
    return EstimateTwoViewInfoUncalibrated(options,
                                           intrinsics1,
                                           intrinsics2,
                                           correspondences,
                                           twoview_info,
                                           inlier_indices);
  }

  // Assume both views are uncalibrated.
  return EstimateTwoViewInfoUncalibrated(options,
                                         intrinsics1,
                                         intrinsics2,
                                         correspondences,
                                         twoview_info,
                                         inlier_indices);
}

//}  // namespace theia
