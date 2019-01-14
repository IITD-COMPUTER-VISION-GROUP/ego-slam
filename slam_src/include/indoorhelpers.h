#ifndef INDOORHELPERS_H
#define INDOORHELPERS_H

#include "ceres_fit.h"
#include "nvmhelpers.h"
#include "pointcloud.h"

enum CorType { straight, corner };

void fitPoints(std::vector<cv::Point3f> &ap, std::vector<cv::Point3f> &fit,
               plane p, float dist) {
  std::vector<cv::Point3f> rem;
  for (int i = 0; i < ap.size(); i++) {
    if (p.distance(ap[i]) < dist) {
      fit.push_back(ap[i]);
    } else {
      rem.push_back(ap[i]);
    }
  }
  ap = rem;
}

void get_roadpoints(std::vector<cv::Point3f> &rd, std::vector<cv::Point3f> &allpt) {
  std::vector<cv::Point3f> rem;
  for (auto it: allpt) {
    if (it.y > 0) {
      rd.push_back(it);
    } else {
      rem.push_back(it);
    }
  }
  allpt = rem;
} 

void get_sidepoints(std::vector<cv::Point3f> &ps, std::vector<cv::Point3f> &rem_points, int side) {
  std::vector<cv::Point3f> rem;
  for (auto it:rem_points) {
    if (side==0) {
      if (it.x>0) {
        ps.push_back(it);
      } else {
        rem.push_back(it);
      }
    } else {
      if (it.x<0) {
        ps.push_back(it);
      } else {
        rem.push_back(it);
      }
    }
  }
  std::cout << ps.size() << "\t is the number of points in side out of " << rem_points.size() << "\n";
  rem_points = rem;
}

void joinpts(std::vector<cv::Point3f> &rem_points, std::vector<cv::Point3f> &bottom_pts) {
  for (auto it:bottom_pts) {
    rem_points.push_back(it);
  }
}

struct corridor {
  CorType ctype;
  float mxdistance;
  plane plane_1;  // roof
  plane plane_2;  // right
  plane plane_3;  // left
  std::vector<cv::Point3f> rem_points;
  std::vector<cv::Point3f> points_1;
  std::vector<cv::Point3f> points_2;
  std::vector<cv::Point3f> points_3;
  std::vector<cv::Point3f> trajectory;
  std::vector<Eigen::Matrix3f> rotations;
  std::vector<double> focals;

  void WriteFile(std::string path) {
    std::ofstream f;
    f.open(path);
    f.precision(8);
    f << trajectory.size() << "\n";
    for (int i = 0; i < trajectory.size(); i++) {
      f << trajectory[i].x << " " << trajectory[i].y << " " << trajectory[i].z
        << "\n"
        << rotations[i](0, 0) << " " << rotations[i](0, 1) << " "
        << rotations[i](0, 2) << " " << rotations[i](1, 0) << " "
        << rotations[i](1, 1) << " " << rotations[i](1, 2) << " "
        << rotations[i](2, 0) << " " << rotations[i](2, 1) << " "
        << rotations[i](2, 2) << "\n";
    }
    f << plane_1.a << " " << plane_1.b << " " << plane_1.c << " " << plane_1.d
      << "\n";
    f << points_1.size() << "\n";
    for (int i = 0; i < points_1.size(); i++) {
      f << points_1[i].x << " " << points_1[i].y << " " << points_1[i].z
        << "\n";
    }
    f << plane_2.a << " " << plane_2.b << " " << plane_2.c << " " << plane_2.d
      << "\n";
    f << points_2.size() << "\n";
    for (int i = 0; i < points_2.size(); i++) {
      f << points_2[i].x << " " << points_2[i].y << " " << points_2[i].z
        << "\n";
    }
    f << plane_3.a << " " << plane_3.b << " " << plane_3.c << " " << plane_3.d
      << "\n";
    f << points_3.size() << "\n";
    for (int i = 0; i < points_3.size(); i++) {
      f << points_3[i].x << " " << points_3[i].y << " " << points_3[i].z
        << "\n";
    }
    f.close();
  }

  void WritePly(std::string path) {
    nvm_file temp;
    for (int i = 0; i < points_1.size(); i++) {
      temp.corr_data.push_back(Corr3D(points_1[i], cv::Point3i(255, 0, 0)));
    }
    for (int i = 0; i < points_2.size(); i++) {
      temp.corr_data.push_back(Corr3D(points_2[i], cv::Point3i(0, 255, 0)));
    }
    for (int i = 0; i < points_3.size(); i++) {
      temp.corr_data.push_back(Corr3D(points_3[i], cv::Point3i(0, 0, 255)));
    }
    for (int i = 0; i < rem_points.size(); i++) {
      temp.corr_data.push_back(Corr3D(rem_points[i], cv::Point3i(0, 0, 0)));
    }
    for (int i = 0; i < trajectory.size(); i++) {
      temp.corr_data.push_back(
          Corr3D(trajectory[i], cv::Point3i(255, 255, 255)));
    }
    temp.save_ply_file(path);
  }

  corridor(){};
  corridor(nvm_file f, int st, int end) {
    std::cout << "Fitting corridor from " << st << " to " << end << "\n";
    mxdistance = f.mdistance_1(st, end);
    // std::cout << mxdistance << " is the distance\n";
    // std::cout << f.distance_kf(st, end - 1) << "\n";
    // f.reset_origin(st);
    f.get_points(st, end, rem_points, trajectory, rotations, focals);
  }

  void initPlanes(CorType c, plane p1, plane p2, plane p3, Eigen::Matrix3f rot,
                  cv::Point3f pos) {
    Eigen::Matrix3f rotT = rot.transpose();

    plane_1 = p1;

    // TODO, add post cornertype
    ctype = c;

    fitPoints(rem_points, points_1, p1, mxdistance * 1.5);
    if (c == straight) {
      fitPoints(rem_points, points_2, p2, mxdistance * 1.5);
      plane_2 = p2;
      fitPoints(rem_points, points_3, p3, mxdistance * 1.5);
      plane_3 = p3;
    } else {
      ctype = straight;
      fitPlane(rem_points, points_2, plane_2, mxdistance);
      fitPlane(rem_points, points_3, plane_3, mxdistance);
    }
  }

  Eigen::Vector3f GetWallNormal() {
    Eigen::Vector3f answer;
    answer(0, 0) = plane_2.a;
    answer(1, 0) = plane_2.b;
    answer(2, 0) = plane_2.c;
    answer.normalize();
    return answer;
  }

  Eigen::Vector3f GetRoofNormal() {
    Eigen::Vector3f answer;
    answer(0, 0) = plane_1.a;
    answer(1, 0) = plane_1.b;
    answer(2, 0) = plane_1.c;
    answer.normalize();
    return answer;
  }

  void basicInit(int tp=0) {
    if (tp==0) {
      ctype = straight;
      plane p1, p2, p3;
      std::vector<cv::Point3f> plane1, plane2, plane3;
      fit3Planes(rem_points, plane1, plane2, plane3, p1, p2, p3,
                 0.8 * mxdistance);
      float dp12 = fabs(p1.dotp(p2));
      float dp13 = fabs(p1.dotp(p3));
      float dp23 = fabs(p2.dotp(p3));
      // std::cout << dp12 << "\t" << dp23 << "\t" << dp13 << "\n";
      if (dp12 > dp23 and dp12 > dp13) {
        // 3 is the roof
        plane_1 = p3;
        points_1 = plane3;
        if (p1.d / p1.a > 0.0) {
          plane_3 = p1;
          plane_2 = p2;
          points_3 = plane1;
          points_2 = plane2;
        } else {
          plane_3 = p2;
          plane_2 = p1;
          points_3 = plane2;
          points_2 = plane1;
        }
      } else if (dp13 > dp23 and dp13 > dp12) {
        // std::cout << "Setting 2 as roof\n";
        // 2 is the roof
        plane_1 = p2;
        points_1 = plane2;
        if (p1.d / p1.a > 0.0) {
          plane_3 = p1;
          plane_2 = p3;
          points_3 = plane1;
          points_2 = plane3;
        } else {
          plane_3 = p3;
          plane_2 = p1;
          points_3 = plane3;
          points_2 = plane1;
        }
      } else {
        // 1 is the roof
        plane_1 = p1;
        points_1 = plane1;
        if (p2.d / p2.a > 0.0) {
          plane_3 = p2;
          plane_2 = p3;
          points_3 = plane2;
          points_2 = plane3;
        } else {
          plane_3 = p3;
          plane_2 = p2;
          points_3 = plane3;
          points_2 = plane2;
        }
      }
    } else {
      // Road case
      mxdistance /=2;
      std::cout << "Fitting a road\n";
      std::vector<cv::Point3f> bottom_pts, side1, side2;
      get_roadpoints(bottom_pts, rem_points);
      fitPlane(bottom_pts, points_1, plane_1, mxdistance);
      joinpts(rem_points, bottom_pts);
      get_sidepoints(side1, rem_points, 0);
      fitPlane(side1, points_2, plane_2, mxdistance, true);
      joinpts(rem_points, side1);
      get_sidepoints(side2, rem_points, 1);
      fitPlane(side2, points_3, plane_3, mxdistance, true);
      joinpts(rem_points, side2);
    }
  }

  void optimise_planes() {
    // Run optimisation pipeline
    if (ctype == straight) {
      optimize(0, plane_1, plane_2, plane_3, points_1, points_2, points_3,
               trajectory, 2 * mxdistance);
    } else {
      // Corner
      return;
      optimize(1, plane_1, plane_2, plane_3, points_1, points_2, points_3,
               trajectory, 2 * mxdistance);
    }
  }

  float get_width() {
    assert(ctype == straight);
    float answer = fabs(plane_2.d - plane_3.d);
    return answer / plane_2.norm();
  }

  float get_height() {
    std::vector<float> distances;
    for (int i = 0; i < trajectory.size(); i++) {
      distances.push_back(plane_1.distance(trajectory[i]));
    }
    std::nth_element(distances.begin(),
                     distances.begin() + (distances.size() / 2),
                     distances.end());
    float answer = distances[distances.size() / 2];
    return answer;
  }

  void scale(float f) {
    plane_1.d *= f;
    plane_2.d *= f;
    plane_3.d *= f;
    for (int i = 0; i < points_1.size(); i++) {
      points_1[i] *= f;
    }
    for (int i = 0; i < points_2.size(); i++) {
      points_2[i] *= f;
    }
    for (int i = 0; i < points_3.size(); i++) {
      points_3[i] *= f;
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] *= f;
    }
    for (int i = 0; i < rem_points.size(); i++) {
      rem_points[i] *= f;
    }
  }

  void shift(cv::Point3f s) {
    plane_1.shift(s);
    plane_2.shift(s);
    plane_3.shift(s);

    for (int i = 0; i < points_1.size(); i++) {
      points_1[i] += s;
    }
    for (int i = 0; i < points_2.size(); i++) {
      points_2[i] += s;
    }
    for (int i = 0; i < points_3.size(); i++) {
      points_3[i] += s;
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] += s;
    }
  }

  static cv::Point3f rotTransform(Eigen::Matrix3f &r, cv::Point3f p) {
    cv::Point3f res;
    res.x = r(0, 0) * p.x + r(0, 1) * p.y + r(0, 2) * p.z;
    res.y = r(1, 0) * p.x + r(1, 1) * p.y + r(1, 2) * p.z;
    res.z = r(2, 0) * p.x + r(2, 1) * p.y + r(2, 2) * p.z;
    return res;
  }

  void rotate(Eigen::Matrix3f rot) {
    plane_1.rotate(rot);
    plane_2.rotate(rot);
    plane_3.rotate(rot);

    for (int i = 0; i < points_1.size(); i++) {
      points_1[i] = rotTransform(rot, points_1[i]);
    }
    for (int i = 0; i < points_2.size(); i++) {
      points_2[i] = rotTransform(rot, points_2[i]);
    }
    for (int i = 0; i < points_3.size(); i++) {
      points_3[i] = rotTransform(rot, points_3[i]);
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] = rotTransform(rot, trajectory[i]);
    }
    for (int i = 0; i < rem_points.size(); i++) {
      rem_points[i] = rotTransform(rot, rem_points[i]);
    }
  }

  Eigen::Vector3f GetDirection() {
    Eigen::Vector3f answer;
    answer(0, 0) = trajectory[trajectory.size() - 1].x - trajectory[0].x;
    answer(1, 0) = trajectory[trajectory.size() - 1].y - trajectory[0].y;
    answer(2, 0) = trajectory[trajectory.size() - 1].z - trajectory[0].z;
    answer.normalize();
    return answer;
  }

  Eigen::Vector3f GetAxis() {
    Eigen::Vector3f n1 = plane_1.GetNormal();
    Eigen::Vector3f n2 = plane_3.GetNormal();
    Eigen::Vector3f a = n1.cross(n2);
    a.normalize();
    Eigen::Vector3f rough = GetDirection();
    float dotp = rough.dot(a);
    if (dotp > 0) {
      return a;
    } else {
      return -a;
    }
  }
};

Eigen::Matrix3f GetRotMatrix(Eigen::Vector3f dest, Eigen::Vector3f src) {
  Eigen::Vector3f v = src.cross(dest);
  float con = src.dot(dest);
  Eigen::Matrix3f vx;
  vx(0, 0) = 0;
  vx(0, 1) = -v(2, 0);
  vx(0, 2) = v(1, 0);
  vx(1, 0) = v(2, 0);
  vx(1, 1) = 0;
  vx(1, 2) = -v(0, 0);
  vx(2, 0) = -v(1, 0);
  vx(2, 1) = v(0, 0);
  vx(2, 2) = 0;
  Eigen::Matrix3f rot;
  rot.setZero();
  rot(0, 0) = 1.0;
  rot(1, 1) = 1.0;
  rot(2, 2) = 1.0;
  rot += vx;
  rot += (1.0 / (1.0 + con)) * vx * vx;
  return rot;
}

void fix_corridor(corridor &c1, corridor &c2, int angle) {
  if (angle == 0) {
    // Straight merging
    std::cout << "Straight merging\n";
    float width2 = c2.get_width();
    float width1 = c1.get_width();
    // std::cout << width1 << "\t" << width2 << "\n";
    c2.scale(width1 / width2);

    Eigen::Vector3f axis1, axis2, roof1, roof2;
    axis1 = c1.GetAxis();
    axis2 = c2.GetAxis();
    if (axis2.dot(axis1) < 0) {
      axis2 *= -1;
    }
    Eigen::Matrix3f rot = GetRotMatrix(axis1, axis2);
    c2.rotate(rot);
    roof1 = c1.GetRoofNormal();
    roof2 = c2.GetRoofNormal();
    if (roof2.dot(roof1) < 0) {
      roof2 *= -1;
    }

    Eigen::Matrix3f rot2 = GetRotMatrix(roof1, roof2);
    c2.rotate(rot2);
    c2.shift(c1.trajectory[c1.trajectory.size() - 1] - c2.trajectory[0]);

  } else {
    // Turn merging
    std::cout << "Turn merging\n";
    float height2 = c2.get_height();
    float height1 = c1.get_height();
    // std::cout << height1 << "\t" << height2 << "\n";
    c2.scale(height1 / height2);
    Eigen::Vector3f axis1, axis2, roof1, roof2;
    axis1 = c1.GetAxis();
    axis2 = c2.GetWallNormal();
    if (axis2.dot(axis1) < 0) {
      axis2 *= -1;
    }
    Eigen::Matrix3f rot = GetRotMatrix(axis1, axis2);
    c2.rotate(rot);

    roof1 = c1.GetRoofNormal();
    roof2 = c2.GetRoofNormal();
    if (roof2.dot(roof1) < 0) {
      roof2 *= -1;
    }
    // std::cout << "Turning Roof 1 " << roof1 << "\n";
    // std::cout << "Turning Roof 2 " << roof2 << "\n";

    Eigen::Matrix3f rot2 = GetRotMatrix(roof1, roof2);
    c2.rotate(rot2);
    c2.shift(c1.trajectory[c1.trajectory.size() - 1] - c2.trajectory[0]);
  }
}

corridor add_corridor(corridor c1, corridor c2) {
  corridor answer(c1);
  for (int i = 0; i < c2.points_1.size(); i++) {
    answer.points_1.push_back(c2.points_1[i]);
  }
  for (int i = 0; i < c2.points_2.size(); i++) {
    answer.points_2.push_back(c2.points_2[i]);
  }
  for (int i = 0; i < c2.points_3.size(); i++) {
    answer.points_3.push_back(c2.points_3[i]);
  }
  for (int i = 0; i < c2.trajectory.size(); i++) {
    answer.trajectory.push_back(c2.trajectory[i]);
  }
  for (int i = 0; i < c2.rem_points.size(); i++) {
    answer.rem_points.push_back(c2.rem_points[i]);
  }
  for (int i = 0; i < c2.focals.size(); i++) {
    answer.focals.push_back(c2.focals[i]);
  }
  for (int i = 0; i < c2.rotations.size(); i++) {
    answer.rotations.push_back(c2.rotations[i]);
  }
  return answer;
}

corridor merge_corridor(corridor c1, corridor c2, int angle) {
  corridor answer(c1);
  // std::cout << "Left wall " << c1.left_wall.dotp(c2.left_wall) << "\n";
  // std::cout << "Right wall " << c1.right_wall.dotp(c2.right_wall) << "\n";
  // std::cout << "Roof " << c1.roof.dotp(c2.roof) << "\n";
  // Scaling
  if (angle == 0) {
    c2.scale(c1.get_width() / c2.get_width());
  } else {
    c2.scale(c1.get_height() / c2.get_height());
  }
  // Rotation
  Eigen::Vector3f axis1, axis2;
  axis1 = c1.GetAxis();
  axis2 = c2.GetAxis();
  // std::cout << "Axis 1" << axis1 << "\n";
  // std::cout << "Axis 2" << axis2 << "\n";
  Eigen::Vector3f v = axis2.cross(axis1);
  float con = axis2.dot(axis1);
  Eigen::Matrix3f vx;
  vx(0, 0) = 0;
  vx(0, 1) = -v(2, 0);
  vx(0, 2) = v(1, 0);
  vx(1, 0) = v(2, 0);
  vx(1, 1) = 0;
  vx(1, 2) = -v(0, 0);
  vx(2, 0) = -v(1, 0);
  vx(2, 1) = v(0, 0);
  vx(2, 2) = 0;
  Eigen::Matrix3f rot;
  rot.setZero();
  rot(0, 0) = 1.0;
  rot(1, 1) = 1.0;
  rot(2, 2) = 1.0;
  rot += vx;
  rot += (1.0 / (1.0 + con)) * vx * vx;
  // std::cout << rot << "\n is the rotation matrix\n";
  c2.rotate(rot);
  axis1 = c1.GetAxis();
  axis2 = c2.GetAxis();
  // std::cout << "Axis 1" << axis1 << "\n";
  // std::cout << "Axis 2" << axis2 << "\n";
  if (angle == 1) {
    // Right turn
  } else if (angle == 2) {
    // Left turn
  }
  // Shifting
  c2.shift(c1.trajectory[c1.trajectory.size() - 1]);
  // std::cout << c1.get_width() << ", " << c2.get_width() << "\n";
  for (int i = 0; i < c2.points_1.size(); i++) {
    answer.points_1.push_back(c2.points_1[i]);
  }
  for (int i = 0; i < c2.points_2.size(); i++) {
    answer.points_2.push_back(c2.points_2[i]);
  }
  for (int i = 0; i < c2.points_3.size(); i++) {
    answer.points_3.push_back(c2.points_3[i]);
  }
  for (int i = 0; i < c2.trajectory.size(); i++) {
    answer.trajectory.push_back(c2.trajectory[i]);
  }

  return answer;
}

#endif
