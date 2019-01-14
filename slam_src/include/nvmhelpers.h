#ifndef NVMHELPERS_H
#define NVMHELPERS_H

#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

struct keyframe_data {
  std::string filename;
  double focal;
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  double d1;
  double d2;
};

struct imgcorr {
  int imgid;
  int siftid;
  cv::Point2f img_location;
};

imgcorr ChangeCamId(imgcorr inp, int x) {
  inp.imgid = x;
  return inp;
}

imgcorr fix_corr(imgcorr inp, int offset) {
  imgcorr result;

  result.imgid = inp.imgid + offset;
  result.siftid = inp.siftid;
  result.img_location = inp.img_location;

  return result;
}

struct Corr3D {
  Eigen::Vector3f point_3d;
  cv::Point3i color;
  std::vector<imgcorr> corr;

  Corr3D(){};
  Corr3D(cv::Point3f p, cv::Point3i c) {
    point_3d(0, 0) = p.x;
    point_3d(1, 0) = p.y;
    point_3d(2, 0) = p.z;
    color = c;
  }

  bool hasInRange(int lower, int upper) {
    for (auto it : corr) {
      if (it.imgid >= lower && it.imgid < upper) return true;
    }
    return false;
  }
};

struct nvm_file {
  std::string description;
  std::vector<keyframe_data> kf_data;
  std::vector<Corr3D> corr_data;
  int median_val;

  nvm_file(){};

  static void q2r(Eigen::Vector4f &q, Eigen::Matrix3f &r) {
    float c2 = q(0, 0);
    r.setZero();
    if (fabs(fabs(c2) - 1.0) > 1e-12) {
      float s2 =
          sqrt(q(1, 0) * q(1, 0) + q(2, 0) * q(2, 0) + q(3, 0) * q(3, 0));
      float s = 2 * s2 * c2;
      float c = 2 * c2 * c2 - 1;
      float cc = 1 - c;
      float n1 = q(1, 0) / s2;
      float n2 = q(2, 0) / s2;
      float n3 = q(3, 0) / s2;
      float n12cc = n1 * n2 * cc;
      float n23cc = n2 * n3 * cc;
      float n31cc = n3 * n1 * cc;
      float n1s = n1 * s;
      float n2s = n2 * s;
      float n3s = n3 * s;

      r(0, 0) = c + n1 * n1 * cc;
      r(0, 1) = n12cc - n3s;
      r(0, 2) = n31cc + n2s;
      r(1, 0) = n12cc + n3s;
      r(1, 1) = c + n2 * c2 * cc;
      r(1, 2) = n23cc - n1s;
      r(2, 0) = n31cc - n2s;
      r(2, 1) = n23cc + n1s;
      r(2, 2) = c + n3 * n3 * cc;

    } else {
      r(0, 0) = 1.0;
      r(1, 1) = 1.0;
      r(2, 2) = 1.0;
    }
  }

  nvm_file(std::string path) {
    std::ifstream nvm_file;
    nvm_file.open(path);
    nvm_file >> description;
    int num_kf, num_corr;
    nvm_file >> num_kf;
    std::cerr << "Number of keyframes " << num_kf << "\n";
    kf_data.resize(num_kf);
    for (int i = 0; i < num_kf; i++) {
      if (description == "NVM_V3") {
        Eigen::Vector4f quat;
        nvm_file >> kf_data[i].filename >> kf_data[i].focal >> quat(0, 0) >>
            quat(1, 0) >> quat(2, 0) >> quat(3, 0) >>
            kf_data[i].translation(0, 0) >> kf_data[i].translation(1, 0) >>
            kf_data[i].translation(2, 0) >> kf_data[i].d1 >> kf_data[i].d2;
        q2r(quat, kf_data[i].rotation);
        kf_data[i].translation = -kf_data[i].rotation * kf_data[i].translation;
      } else {
        nvm_file >> kf_data[i].filename >> kf_data[i].focal >>
            kf_data[i].rotation(0, 0) >> kf_data[i].rotation(0, 1) >>
            kf_data[i].rotation(0, 2) >> kf_data[i].rotation(1, 0) >>
            kf_data[i].rotation(1, 1) >> kf_data[i].rotation(1, 2) >>
            kf_data[i].rotation(2, 0) >> kf_data[i].rotation(2, 1) >>
            kf_data[i].rotation(2, 2) >> kf_data[i].translation(0, 0) >>
            kf_data[i].translation(1, 0) >> kf_data[i].translation(2, 0) >>
            kf_data[i].d1 >> kf_data[i].d2;
      }
    }
    if (description == "NVM_V3") {
      description = "NVM_V3_R9T";
    }
    nvm_file >> num_corr;
    std::cerr << "Number of correspondances " << num_corr << "\n";
    corr_data.resize(num_corr);
    std::vector<int> all_corrs_counts;
    for (int i = 0; i < num_corr; i++) {
      nvm_file >> corr_data[i].point_3d(0, 0) >> corr_data[i].point_3d(1, 0) >>
          corr_data[i].point_3d(2, 0) >> corr_data[i].color.x >>
          corr_data[i].color.y >> corr_data[i].color.z;
      int num_2d;
      nvm_file >> num_2d;
      all_corrs_counts.push_back(num_2d);
      corr_data[i].corr.resize(num_2d);
      for (int j = 0; j < num_2d; j++) {
        nvm_file >> corr_data[i].corr[j].imgid >> corr_data[i].corr[j].siftid >>
            corr_data[i].corr[j].img_location.x >>
            corr_data[i].corr[j].img_location.y;
      }
    }
    std::cout << "Number of corrs: " << corr_data.size() << "\n";
    nvm_file.close();
    std::nth_element(all_corrs_counts.begin(),
                     all_corrs_counts.begin() + all_corrs_counts.size() / 2,
                     all_corrs_counts.end());
    median_val = all_corrs_counts[all_corrs_counts.size() / 2];
    std::cout << median_val << "\n";
  }

  int get_imgid(int ind) {
    int st = 0, end = 0;
    std::string fname = kf_data[ind].filename;
    st = 1 + fname.find("_");
    end = fname.find(".");
    return std::stoi(fname.substr(st, end));
  }

  std::string get_imgtyp() {
    return kf_data[0].filename.substr(kf_data[0].filename.find("."));
  }

  void sortNVM() {
    std::unordered_map<std::string, int> old_invmap;
    std::unordered_map<int, std::string> old_map;
    std::unordered_map<std::string, int> new_map;
    std::vector<int> all_files;
    for (int i = 0; i < kf_data.size(); i++) {
      old_map[i] = kf_data[i].filename;
      old_invmap[kf_data[i].filename] = i;
      all_files.push_back(get_imgid(i));
    }
    std::sort(all_files.begin(), all_files.end());
    std::vector<keyframe_data> new_kf;
    for (int i = 0; i < all_files.size(); i++) {
      new_kf.push_back(
          kf_data[old_invmap["img_" + std::to_string(all_files[i]) +
                             get_imgtyp()]]);
      new_map["img_" + std::to_string(all_files[i]) + get_imgtyp()] = i;
    }
    kf_data = new_kf;
    for (int i = 0; i < corr_data.size(); i++) {
      for (int j = 0; j < corr_data[i].corr.size(); j++) {
        corr_data[i].corr[j].imgid =
            new_map[old_map[corr_data[i].corr[j].imgid]];
      }
    }
  }

  void save_to_disk(std::string path) {
    std::cout << "Saving " << path << "\n";
    std::ofstream nvmfile;
    nvmfile.open(path);
    nvmfile.precision(8);
    nvmfile << description << "\n";
    nvmfile << kf_data.size() << "\n";
    for (int i = 0; i < kf_data.size(); i++) {
      nvmfile << kf_data[i].filename << " " << kf_data[i].focal << " "
              << kf_data[i].rotation(0, 0) << " " << kf_data[i].rotation(0, 1)
              << " " << kf_data[i].rotation(0, 2) << " "
              << kf_data[i].rotation(1, 0) << " " << kf_data[i].rotation(1, 1)
              << " " << kf_data[i].rotation(1, 2) << " "
              << kf_data[i].rotation(2, 0) << " " << kf_data[i].rotation(2, 1)
              << " " << kf_data[i].rotation(2, 2) << " "
              << kf_data[i].translation(0, 0) << " "
              << kf_data[i].translation(1, 0) << " "
              << kf_data[i].translation(2, 0) << " " << kf_data[i].d1 << " "
              << kf_data[i].d2 << "\n";
    }
    nvmfile << corr_data.size() << "\n";
    for (int i = 0; i < corr_data.size(); i++) {
      nvmfile << corr_data[i].point_3d(0, 0) << " "
              << corr_data[i].point_3d(1, 0) << " "
              << corr_data[i].point_3d(2, 0) << " " << corr_data[i].color.x
              << " " << corr_data[i].color.y << " " << corr_data[i].color.z
              << " " << corr_data[i].corr.size() << " ";
      for (int j = 0; j < corr_data[i].corr.size(); j++) {
        nvmfile << corr_data[i].corr[j].imgid << " "
                << corr_data[i].corr[j].siftid << " "
                << corr_data[i].corr[j].img_location.x << " "
                << corr_data[i].corr[j].img_location.y << " ";
      }
      nvmfile << "\n";
    }
    nvmfile.close();
  }

  int num_kf() { return kf_data.size(); }

  Eigen::Vector3f get_camera_position(int i) {
    return -kf_data[i].rotation.transpose() * kf_data[i].translation;
  }

  void save_ply_file(std::string path) {
    std::ofstream plyfile;
    plyfile.open(path);
    plyfile.precision(8);
    plyfile << "ply\nformat ascii 1.0\nelement vertex "
            << corr_data.size() + kf_data.size()
            << "\nproperty float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar "
               "blue\nend_header\n";
    for (int i = 0; i < corr_data.size(); i++) {
      plyfile << corr_data[i].point_3d(0, 0) << " "
              << corr_data[i].point_3d(1, 0) << " "
              << corr_data[i].point_3d(2, 0) << " " << corr_data[i].color.x
              << " " << corr_data[i].color.y << " " << corr_data[i].color.z
              << "\n";
    }
    for (int i = 0; i < kf_data.size(); i++) {
      Eigen::Vector3f temp =
          -kf_data[i].rotation.transpose() * kf_data[i].translation;
      plyfile << temp(0, 0) << " " << temp(1, 0) << " " << temp(2, 0)
              << " 255 255 255\n";
    }
    plyfile.close();
  }

  void save_focal_for_optimisation(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    ffile << kf_data.size() << "\n";
    for (int i = 0; i < kf_data.size(); i++) {
      ffile << kf_data[i].filename << " " << kf_data[i].focal << "\n";
    }
    ffile.close();
  }

  void save_rt_global_file(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i = 0; i < kf_data.size(); i++) {
      Eigen::Vector3f temp =
          -kf_data[i].rotation.transpose() * kf_data[i].translation;
      ffile << kf_data[i].rotation(0, 0) << " " << kf_data[i].rotation(0, 1)
            << " " << kf_data[i].rotation(0, 2) << " "
            << kf_data[i].rotation(1, 0) << " " << kf_data[i].rotation(1, 1)
            << " " << kf_data[i].rotation(1, 2) << " "
            << kf_data[i].rotation(2, 0) << " " << kf_data[i].rotation(2, 1)
            << " " << kf_data[i].rotation(2, 2) << "\n"
            << temp(0, 0) << " " << temp(1, 0) << " " << temp(2, 0) << "\n";
    }
    ffile.close();
  }

  void save_distortion_file(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i = 0; i < kf_data.size(); i++) {
      ffile << kf_data[i].d1 << "\n";
    }
    ffile.close();
  }

  void save_invmap(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i = 0; i < kf_data.size(); i++) {
      ffile << i << " " << kf_data[i].filename << "\n";
    }
    ffile.close();
  }

  void save_ours_new(std::string path) {
    std::ofstream nvmfile;
    nvmfile.open(path);
    for (int i = 0; i < corr_data.size(); i++) {
      nvmfile << corr_data[i].point_3d(0, 0) << " "
              << corr_data[i].point_3d(1, 0) << " "
              << corr_data[i].point_3d(2, 0) << " " << corr_data[i].color.x
              << " " << corr_data[i].color.y << " " << corr_data[i].color.z
              << " " << corr_data[i].corr.size() << " ";
      for (int j = 0; j < corr_data[i].corr.size(); j++) {
        nvmfile << corr_data[i].corr[j].imgid << " "
                << corr_data[i].corr[j].siftid << " "
                << corr_data[i].corr[j].img_location.x << " "
                << corr_data[i].corr[j].img_location.y << " ";
      }
      nvmfile << "\n";
    }
    nvmfile.close();
  }

  Eigen::Vector3f get_translation_vector(int f1, int f2) {
    Eigen::Vector3f p1 = get_camera_position(f1);
    Eigen::Vector3f p2 = get_camera_position(f2);
    Eigen::Vector3f ans = p2-p1;
    std::cout << ans << "\n";
    ans.normalize();
    return ans;
  }

  void get_points(int lowerbound, int upperbound,
                  std::vector<cv::Point3f> &pointsfound,
                  std::vector<cv::Point3f> &trajectory,
                  std::vector<Eigen::Matrix3f> &rotations,
                  std::vector<double> &focals) {
    pointsfound.clear();
    trajectory.clear();
    rotations.clear();
    focals.clear();
    for (auto it : corr_data) {
      if (it.hasInRange(lowerbound, upperbound)) {
        pointsfound.push_back(cv::Point3f(it.point_3d(0, 0), it.point_3d(1, 0),
                                          it.point_3d(2, 0)));
      }
    }
    for (int i = lowerbound; i < upperbound; i++) {
      Eigen::Vector3f location =
          -kf_data[i].rotation.transpose() * kf_data[i].translation;
      trajectory.push_back(
          cv::Point3f(location(0, 0), location(1, 0), location(2, 0)));
      rotations.push_back(kf_data[i].rotation);
      focals.push_back(kf_data[i].focal);
    }
  }

  void rotate(Eigen::Matrix3f rot, Eigen::Matrix3f rotT) {
    for (int i = 0; i < corr_data.size(); i++) {
      corr_data[i].point_3d = rot * corr_data[i].point_3d;
    }
    for (int i = 0; i < kf_data.size(); i++) {
      kf_data[i].translation =
          -kf_data[i].rotation.transpose() * kf_data[i].translation;
      kf_data[i].translation = rot * kf_data[i].translation;
      kf_data[i].rotation = kf_data[i].rotation * rotT;
      kf_data[i].translation = -kf_data[i].rotation * kf_data[i].translation;
    }
  }

  void reset_origin(int index) {
    Eigen::Matrix3f rotT = kf_data[index].rotation.transpose();
    Eigen::Matrix3f rot = kf_data[index].rotation.transpose();
    rotate(rot, rotT);
    Eigen::Vector3f location =
        -kf_data[index].rotation.transpose() * kf_data[index].translation;
    for (int i = 0; i < corr_data.size(); i++) {
      corr_data[i].point_3d -= location;
    }
    for (int i = 0; i < kf_data.size(); i++) {
      kf_data[i].translation =
          kf_data[i].translation + kf_data[i].rotation * location;
    }
  }

  void reset_coord(int f1, int f2) {
    Eigen::Vector3f t1 = get_translation_vector(f1, f2);
    std::cout << t1 << "\n";
    float dotp = t1(2,0);
    float theta = acos(dotp);
    Eigen::Matrix3f rmat;
    rmat.setZero();
    rmat(0,0) = 1.0;
    rmat(1,1) = cos(theta);
    rmat(2,2) = cos(theta);
    rmat(1,2) = -sin(theta);
    rmat(2,1) = sin(theta);
    std::cout << rmat << "\n";
    rotate(rmat, rmat.transpose());
  }

  float mdistance() {
    float maxx(corr_data[0].point_3d(0, 0)), maxy(corr_data[0].point_3d(1, 0)),
        maxz(corr_data[0].point_3d(2, 0)), minx(corr_data[0].point_3d(0, 0)),
        miny(corr_data[0].point_3d(1, 0)), minz(corr_data[0].point_3d(2, 0));

    for (auto it : corr_data) {
      maxx = std::max(maxx, it.point_3d(0, 0));
      maxy = std::max(maxy, it.point_3d(1, 0));
      maxz = std::max(maxz, it.point_3d(2, 0));
      minx = std::min(minx, it.point_3d(0, 0));
      miny = std::min(miny, it.point_3d(1, 0));
      minz = std::min(minz, it.point_3d(2, 0));
    }

    return sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) +
                (maxz - minz) * (maxz - minz));
  }

  float distance_kf(int f1, int f2) {
    Eigen::Vector3f p1 = get_camera_position(f1);
    Eigen::Vector3f p2 = get_camera_position(f2);
    return (p2 - p1).norm();
  }

  float mdistance_1(int st, int end) {
    float ans(0.0);
    for (int i = st; i < end - 1; i++) {
      ans += distance_kf(i, i + 1);
    }
    return (ans / (end - st - 1));
  }

  Eigen::Vector3f get_motion_vector(int f1, int f2, int ref) {
    // TODO incorporate reference frame
    Eigen::Vector3f p1, p2;
    p1 = get_camera_position(f1);
    p2 = get_camera_position(f2);
    Eigen::Vector3f result = p2 - p1;
    result.normalize();
    return result;
  }

  float getdotp(int i1, int i2, int i3) {
    Eigen::Vector3f m1 = get_motion_vector(i1, i2, 0);
    Eigen::Vector3f m2 = get_motion_vector(i2, i3, 0);
    return fabs(m1.dot(m2));
  }
};

float get_best_scaling_factor(nvm_file &f1, nvm_file &f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it : f1.corr_data) {
    m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it : f2.corr_data) {
    if (m1.find(it.corr[0].siftid) != m1.end()) {
      // std::cout << it.corr[0].siftid << "\n";
      // std::cout << m1[it.corr[0].siftid] << "\n";
      points_common.push_back(
          std::make_pair(m1[it.corr[0].siftid], it.point_3d));
    }
  }
  srand(time(NULL));
  float d1(0), d2(0);
  int limt = points_common.size();
  for (int i = 0; i < 500; i++) {
    int p1 = rand() % limt;
    int p2 = rand() % limt;
    // std::cout << p1 << "\t" << p2 <<"\n";
    // std::cout << points_common[p1].first << "\t" << points_common[p2].first
    // << "\n";
    d1 += (points_common[p1].first - points_common[p2].first).norm();
    d2 += (points_common[p1].second - points_common[p2].second).norm();
  }
  std::cerr << "Number of common points " << points_common.size() << "\n";
  std::cerr << d1 << "\t" << d2 << "\t" << d1 / d2 << "\n";
  return d1 / d2;
}

void get_best_translation(nvm_file &f1, nvm_file &f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it : f1.corr_data) {
    m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it : f2.corr_data) {
    if (m1.find(it.corr[0].siftid) != m1.end()) {
      // std::cout << it.corr[0].siftid << "\n";
      // std::cout << m1[it.corr[0].siftid] << "\n";
      points_common.push_back(
          std::make_pair(m1[it.corr[0].siftid], it.point_3d));
    }
  }

  Eigen::Vector3f answer1, answer2;
  answer1.setZero();
  answer2.setZero();

  for (auto it : points_common) {
    answer1 += it.first;
    answer2 += it.second;
  }
  answer1(0, 0) /= points_common.size();
  answer1(1, 0) /= points_common.size();
  answer1(2, 0) /= points_common.size();

  answer2(0, 0) /= points_common.size();
  answer2(1, 0) /= points_common.size();
  answer2(2, 0) /= points_common.size();

  std::cerr << answer1 << "\n";
  std::cerr << answer2 << "\n";

  for (Corr3D &it : f1.corr_data) {
    it.point_3d -= answer1;
    // it.color.x=255;
    // it.color.y=0;
    // it.color.z=0;
  }
  for (Corr3D &it : f2.corr_data) {
    it.point_3d -= answer2;
    // it.color.x=0;
    // it.color.y=0;
    // it.color.z=255;
  }
  // std::cout << answer << "\n";
  // return answer;
}

void GetBestRST(nvm_file &f1, nvm_file &f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it : f1.corr_data) {
    if (it.corr.size() >= 0) m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it : f2.corr_data) {
    if (it.corr.size() >= 0) {
      if (m1.find(it.corr[0].siftid) != m1.end()) {
        points_common.push_back(
            std::make_pair(m1[it.corr[0].siftid], it.point_3d));
      }
    }
  }

  std::cout << "Number of points " << points_common.size() << "\n";

  Eigen::Vector3f answer1, answer2;
  answer1.setZero();
  answer2.setZero();

  for (auto it : points_common) {
    answer1 += it.first;
    answer2 += it.second;
  }
  answer1(0, 0) /= points_common.size();
  answer1(1, 0) /= points_common.size();
  answer1(2, 0) /= points_common.size();

  answer2(0, 0) /= points_common.size();
  answer2(1, 0) /= points_common.size();
  answer2(2, 0) /= points_common.size();

  Eigen::MatrixXf points1 = Eigen::MatrixXf(3, points_common.size());
  Eigen::MatrixXf points2 = Eigen::MatrixXf(3, points_common.size());

  for (int i = 0; i < points_common.size(); i++) {
    points1(0, i) = points_common[i].first(0, 0) - answer1(0, 0);
    points1(1, i) = points_common[i].first(1, 0) - answer1(1, 0);
    points1(2, i) = points_common[i].first(2, 0) - answer1(2, 0);
    points2(0, i) = points_common[i].second(0, 0) - answer2(0, 0);
    points2(1, i) = points_common[i].second(1, 0) - answer2(1, 0);
    points2(2, i) = points_common[i].second(2, 0) - answer2(2, 0);
  }

  Eigen::MatrixXf r(Eigen::MatrixXf::Identity(3, 3));
  float scale = 1;
  Eigen::Vector3f t;
  t.setZero();
  if (points_common.size() >= 3) {
    Eigen::MatrixXf temp = points1 * (points2.transpose());
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        temp, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXf s = Eigen::MatrixXf::Identity(3, 3);
    std::cout << svd.singularValues() << "\n";
    r = svd.matrixU() * svd.matrixV().transpose();
    if (r.determinant() < 0) {
      std::cerr << "Determinant obtained < 0\n";
      int minsofar = 0;
      float minval = r(0, 0);
      if (r(1, 1) < minval) {
        minval = r(1, 1);
        minsofar = 1;
      }
      if (r(2, 2) < minval) {
        minval = r(2, 2);
        minsofar = 2;
      }
      s(minsofar, minsofar) = -1;
      r = svd.matrixU() * s * svd.matrixV().transpose();
    }

    scale = s(0, 0) * svd.singularValues()(0, 0) +
            s(1, 1) * svd.singularValues()(1, 0) +
            s(2, 2) * svd.singularValues()(2, 0);

    float denom = points2.squaredNorm();
    scale /= denom;

    t = answer1 - scale * r * answer2;
  }

  // scale = 16.0;
  // t << 0.137, -0.145, 0.86;

  // r << 0.6431,         0,   -0.7658,
  //     -0.1065,    0.9903,   -0.0895,
  //      0.7583,    0.1391,    0.6369;

  std::cout << "scale: " << scale << "\n";
  std::cout << "rotation " << r << "\n";
  std::cout << "trans " << t << "\n";
  for (int i = 0; i < f2.corr_data.size(); i++) {
    f2.corr_data[i].point_3d = scale * r * f2.corr_data[i].point_3d + t;
  }

  for (int i = 0; i < f2.kf_data.size(); i++) {
    f2.kf_data[i].rotation = f2.kf_data[i].rotation * r.transpose();
    f2.kf_data[i].translation =
        scale * f2.kf_data[i].translation - f2.kf_data[i].rotation * t;
  }
}

nvm_file merge_nvm(nvm_file &f1, nvm_file &f2) {
  nvm_file output;
  assert(f1.description == f2.description);
  output.description = f1.description;
  for (int i = 0; i < f1.kf_data.size(); i++) {
    output.kf_data.push_back(f1.kf_data[i]);
  }
  int offset = 0;
  if (f1.kf_data.rbegin()->filename == f2.kf_data.begin()->filename) {
    offset = 1;
  }
  for (int i = offset; i < f2.kf_data.size(); i++) {
    output.kf_data.push_back(f2.kf_data[i]);
  }

  std::unordered_map<int, Corr3D> all_mappings;
  for (int i = 0; i < f1.corr_data.size(); i++) {
    all_mappings[f1.corr_data[i].corr[0].siftid] = f1.corr_data[i];
  }
  offset = f1.kf_data.size() - offset;

  for (int i = 0; i < f2.corr_data.size(); i++) {
    int sftid = f2.corr_data[i].corr[0].siftid;
    if (all_mappings.find(sftid) == all_mappings.end()) {
      all_mappings[sftid].point_3d = f2.corr_data[i].point_3d;
      all_mappings[sftid].color = f2.corr_data[i].color;
      for (int j = 0; j < f2.corr_data[i].corr.size(); j++) {
        all_mappings[sftid].corr.push_back(
            fix_corr(f2.corr_data[i].corr[j], offset));
      }
    } else {
      // sift id existed
      for (int j = 0; j < f2.corr_data[i].corr.size(); j++) {
        if (f2.corr_data[i].corr[j].imgid > 0) {
          all_mappings[sftid].corr.push_back(
              fix_corr(f2.corr_data[i].corr[j], offset));
        }
      }
    }
  }

  for (auto it : all_mappings) {
    output.corr_data.push_back(it.second);
  }
  std::cout << "Number of points " << output.corr_data.size() << "\n";

  return output;
}

#endif
