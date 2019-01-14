// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

#include "ceres_fit.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

void optimize(int type, plane& rf, plane& lft, plane& rt,
              std::vector<cv::Point3f>& pts_rf,
              std::vector<cv::Point3f>& pts_left,
              std::vector<cv::Point3f>& pts_right,
              std::vector<cv::Point3f>& trajectory, float mindistance) {
  double a = -lft.a / lft.d;
  double b = -lft.b / lft.d;
  double c = -lft.c / lft.d;
  double d = lft.d / lft.d;
  double e = rt.d / lft.d;
  double k = -rf.a / rf.d;
  double l = -rf.b / rf.d;
  double m = -rf.c / rf.d;
  double n = rf.d / rf.d;
  Problem problem;

  cv::Point3f trac;
  trac = trajectory[trajectory.size() - 1] - trajectory[0];

  for (int i = 0; i < pts_left.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_left[i].x, (double)pts_left[i].y,
                               (double)pts_left[i].z)),
        NULL, &a, &b, &c, &d);
  }
  for (int i = 0; i < pts_right.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_right[i].x, (double)pts_right[i].y,
                               (double)pts_right[i].z)),
        NULL, &a, &b, &c, &e);
  }
  for (int i = 0; i < pts_rf.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_rf[i].x, (double)pts_rf[i].y,
                               (double)pts_rf[i].z)),
        NULL, &k, &l, &m, &n);
  }
  problem.AddResidualBlock(
      new AutoDiffCostFunction<PerpResidual, 1, 1, 1, 1, 1, 1, 1>(
          new PerpResidual()),
      NULL, &a, &b, &c, &k, &l, &m);

  problem.AddResidualBlock(
      new AutoDiffCostFunction<PerpTrajectoryResidual, 1, 1, 1, 1>(
          new PerpTrajectoryResidual((double)trac.x, (double)trac.y,
                                     (double)trac.z)),
      NULL, &k, &l, &m);

  problem.SetParameterBlockConstant(&d);
  problem.SetParameterBlockConstant(&n);
  Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = true;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  // std::cout << "Final 1 " << a << "\t" << b << "\t" << c << "\t" << d << "\t"
  //           << e << "\n";
  // std::cout << "Final 2 " << k << "\t" << l << "\t" << m << "\t" << n <<
  // "\n";
  // std::cout << "Error: "
  //           << (a * k + b * l + c * m) /
  //                  sqrt((a * a + b * b + c * c) * (k * k + l * l + m * m))
  //           << "\n";
  rf.a = k;
  rf.b = l;
  rf.c = m;
  rf.d = -n;
  rf.normalize();
  lft.a = a;
  lft.b = b;
  lft.c = c;
  lft.d = -d;
  lft.normalize();
  rt.a = a;
  rt.b = b;
  rt.c = c;
  rt.d = -e;
  rt.normalize();

  // return;

  // Iteration 2
  std::vector<cv::Point3f> newroof, newleft, newright;
  for (int i = 0; i < pts_rf.size(); i++) {
    float v1, v2, v3;
    v1 = fabs(rf.value(pts_rf[i]));
    v2 = fabs(lft.value(pts_rf[i]));
    v3 = fabs(rt.value(pts_rf[i]));

    if (v1 <= v2 and v1 <= v3 and v1 <= mindistance) {
      newroof.push_back(pts_rf[i]);
    } else if (v2 <= v1 and v2 <= v3 and v2 <= mindistance) {
      newleft.push_back(pts_rf[i]);
    } else if (v3 <= v1 and v3 <= v2 and v3 <= mindistance) {
      newright.push_back(pts_rf[i]);
    }
  }
  for (int i = 0; i < pts_left.size(); i++) {
    float v1, v2, v3;
    v1 = fabs(rf.value(pts_left[i]));
    v2 = fabs(lft.value(pts_left[i]));
    v3 = fabs(rt.value(pts_left[i]));

    if (v1 <= v2 and v1 <= v3 and v1 <= mindistance) {
      newroof.push_back(pts_left[i]);
    } else if (v2 <= v1 and v2 <= v3 and v2 <= mindistance) {
      newleft.push_back(pts_left[i]);
    } else if (v3 <= v1 and v3 <= v2 and v3 <= mindistance) {
      newright.push_back(pts_left[i]);
    }
  }
  for (int i = 0; i < pts_right.size(); i++) {
    float v1, v2, v3;
    v1 = fabs(rf.value(pts_right[i]));
    v2 = fabs(lft.value(pts_right[i]));
    v3 = fabs(rt.value(pts_right[i]));

    if (v1 <= v2 and v1 <= v3 and v1 <= mindistance) {
      newroof.push_back(pts_right[i]);
    } else if (v2 <= v1 and v2 <= v3 and v2 <= mindistance) {
      newleft.push_back(pts_right[i]);
    } else if (v3 <= v1 and v3 <= v2 and v3 <= mindistance) {
      newright.push_back(pts_right[i]);
    }
  }

  // std::cout << "Left points changed from " << pts_left.size() << " to ";
  pts_left = newleft;
  // std::cout << pts_left.size() << "\n";
  pts_right = newright;
  pts_rf = newroof;

  Problem problem2;

  for (int i = 0; i < pts_left.size(); ++i) {
    problem2.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_left[i].x, (double)pts_left[i].y,
                               (double)pts_left[i].z)),
        NULL, &a, &b, &c, &d);
  }
  for (int i = 0; i < pts_right.size(); ++i) {
    problem2.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_right[i].x, (double)pts_right[i].y,
                               (double)pts_right[i].z)),
        NULL, &a, &b, &c, &e);
  }
  for (int i = 0; i < pts_rf.size(); ++i) {
    problem2.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_rf[i].x, (double)pts_rf[i].y,
                               (double)pts_rf[i].z)),
        NULL, &k, &l, &m, &n);
  }
  problem2.AddResidualBlock(
      new AutoDiffCostFunction<PerpResidual, 1, 1, 1, 1, 1, 1, 1>(
          new PerpResidual()),
      NULL, &a, &b, &c, &k, &l, &m);
  problem2.AddResidualBlock(
      new AutoDiffCostFunction<PerpTrajectoryResidual, 1, 1, 1, 1>(
          new PerpTrajectoryResidual((double)trac.x, (double)trac.y,
                                     (double)trac.z)),
      NULL, &k, &l, &m);

  problem2.SetParameterBlockConstant(&d);
  problem2.SetParameterBlockConstant(&n);
  Solver::Options options2;
  options2.max_num_iterations = 1000;
  options2.linear_solver_type = ceres::DENSE_QR;
  options2.use_nonmonotonic_steps = true;
  options2.minimizer_progress_to_stdout = false;
  Solver::Summary summary2;
  Solve(options2, &problem2, &summary2);
  // std::cout << summary.FullReport() << "\n";
  // std::cout << "Final 1 " << a << "\t" << b << "\t" << c << "\t" << d << "\t"
  //           << e << "\n";
  // std::cout << "Final 2 " << k << "\t" << l << "\t" << m << "\t" << n <<
  // "\n";
  // std::cout << "Error: "
  //           << (a * k + b * l + c * m) /
  //                  sqrt((a * a + b * b + c * c) * (k * k + l * l + m * m))
  //           << "\n";
  rf.a = k;
  rf.b = l;
  rf.c = m;
  rf.d = -n;
  rf.normalize();
  lft.a = a;
  lft.b = b;
  lft.c = c;
  lft.d = -d;
  lft.normalize();
  rt.a = a;
  rt.b = b;
  rt.c = c;
  rt.d = -e;
  rt.normalize();
}
