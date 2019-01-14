#ifndef INTERMEDIATE_HELPERS_H
#define INTERMEDIATE_HELPERS_H

#include "nvmhelpers.h"
#include <unordered_set>
#include "triangulate.h"

struct intermediate_chunk {
  int f1;
  int f2;
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  std::vector<Corr3D> all_cors; 
};

struct intermediate_file {
  std::vector<intermediate_chunk> all_data;

  intermediate_file() {};
  intermediate_file(std::string path) {
    std::ifstream interfile;
    interfile.open(path);
    int f1, f2;
    while (interfile >> f1 >> f2) {
      intermediate_chunk temp;
      temp.f1 = f1;
      temp.f2 = f2;
      interfile >> temp.rotation(0,0) >> temp.rotation(0,1) >> temp.rotation(0,2)
                >> temp.rotation(1,0) >> temp.rotation(1,1) >> temp.rotation(1,2)
                >> temp.rotation(2,0) >> temp.rotation(2,1) >> temp.rotation(2,2);
      interfile >> temp.translation(0,0) >> temp.translation(1,0) >> temp.translation(2,0);
      int ct;
      interfile >> ct;
      temp.all_cors.resize(ct);
      for (int i=0; i<temp.all_cors.size(); i++) {
        temp.all_cors[i].corr.resize(2);
        interfile >> temp.all_cors[i].color.x >> temp.all_cors[i].color.y >> temp.all_cors[i].color.z;
        interfile >> temp.all_cors[i].corr[0].siftid >> temp.all_cors[i].corr[0].img_location.x >> temp.all_cors[i].corr[0].img_location.y;
        interfile >> temp.all_cors[i].corr[1].siftid >> temp.all_cors[i].corr[1].img_location.x >> temp.all_cors[i].corr[1].img_location.y;
        assert(temp.all_cors[i].corr[0].siftid == temp.all_cors[i].corr[1].siftid);
        temp.all_cors[i].corr[0].imgid = f1;
        temp.all_cors[i].corr[1].imgid = f2;
      }
      all_data.push_back(temp);
    }
    interfile.close();
  }
};

void add_intermediate_points(nvm_file& input, intermediate_file& infile,
  std::vector<std::string> &focal_file) {
  std::unordered_set<int> all_sift_ids;
  for (int i=0; i<input.corr_data.size(); i++) {
    all_sift_ids.insert(input.corr_data[i].corr[0].siftid);
  }
  std::unordered_map<int, std::unordered_map<int, imgcorr> > new_cors;
  std::unordered_map<int, cv::Point3i> new_cols;
  for (int i=0; i<infile.all_data.size(); i++) {
    for (int j=0; j<infile.all_data[i].all_cors.size(); j++) {
      int sftid = infile.all_data[i].all_cors[j].corr[0].siftid;
      if (all_sift_ids.find(sftid)==all_sift_ids.end()) {
        new_cols[sftid] = infile.all_data[i].all_cors[j].color; 
        new_cors[sftid][infile.all_data[i].f1] = infile.all_data[i].all_cors[j].corr[0];
        new_cors[sftid][infile.all_data[i].f2] = infile.all_data[i].all_cors[j].corr[1];
      }
    }
  }
  std::unordered_map<std::string, int> inverse_map;
  for (int i=0; i<input.kf_data.size(); i++) {
    inverse_map[input.kf_data[i].filename] = i;
  }
  std::cout << "Created inverse map\n";
  // input.corr_data.clear();
  // Assuming no camera deletions
  for (auto it: new_cors) {
    std::vector<triangulation_bundle> to_triangulate;
    for (auto it1 : it.second) {
      // std::cout << it1.first << "\n"; 
      if (inverse_map.find(focal_file[it1.first]) == inverse_map.end()) {
        std::cerr << "Camera had got deleted :(\n";
        continue;
      }
      int cam_id = inverse_map[focal_file[it1.first]];
      // std::cout << it1.first << "\t" << cam_id << "\n"; 
      assert(cam_id < input.kf_data.size()); 
      to_triangulate.push_back(triangulation_bundle(
        camera_frame_wo_image(input.kf_data[cam_id].focal, 
                              input.kf_data[cam_id].rotation,
                              input.kf_data[cam_id].translation,
                              0, 0),
            it1.second.img_location));
    }
    if (to_triangulate.size() > 2) {
      // std::cout << Triangulate(to_triangulate) << "\n";
      input.corr_data.push_back(Corr3D(Triangulate(to_triangulate), new_cols[it.first]));
      for (auto it1: it.second) {
        input.corr_data.rbegin()->corr.push_back(ChangeCamId(it1.second,inverse_map[focal_file[it1.first]]));
      }
    }
  }

};

#endif
