// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#ifndef THEIA_SOLVERS_INLIER_SUPPORT_H_
#define THEIA_SOLVERS_INLIER_SUPPORT_H_

#include <algorithm>
#include <vector>

#include "quality_measurement.h"

//namespace theia {
// Assess quality of data by whether each error residual is less than an error
// threshold. If it is below the threshold, it is considered an inlier.
class InlierSupport : public QualityMeasurement {
 public:
  explicit InlierSupport(const double error_thresh)
      : QualityMeasurement(error_thresh) {}
  ~InlierSupport() {}

  bool Initialize()  {
    max_inlier_ratio_ = 0.0;
    return true;
  }

  // Count the number of inliers in the data and return the cost such that lower
  // cost is better.
  double ComputeCost(const std::vector<double>& residuals) {
    double num_inliers = 0.0;
    for (int i = 0; i < residuals.size(); i++) {
      if (residuals[i] < this->error_thresh_) {
        num_inliers += 1.0;
      }
    }
    const double inlier_ratio =
        num_inliers / static_cast<double>(residuals.size());
    max_inlier_ratio_ = std::max(inlier_ratio, max_inlier_ratio_);
    return residuals.size() - num_inliers;
  }

  double GetInlierRatio() const { return max_inlier_ratio_; }

 private:
  double max_inlier_ratio_;
};

//}  // namespace theia

#endif  // THEIA_SOLVERS_INLIER_SUPPORT_H_
