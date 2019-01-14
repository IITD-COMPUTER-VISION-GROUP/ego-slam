#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_map>
#include <fstream>

struct imgcorr {
  int siftid;
  cv::Point2f imglocation;

  imgcorr() {};

  imgcorr(int sift, float p1, float p2) {
    siftid = sift;
    imglocation.x = p1;
    imglocation.y = p2;
  }
};

struct world_corr {
  cv::Point3f location;
  cv::Vec3b color;
  // img id to match
  std::unordered_map<int, imgcorr> pairs;

  world_corr() {};
};

struct total_world {
  // sift id to matches
  std::unordered_map<int, world_corr> all_matches;

  total_world() {};

  total_world(std::string data_dir, std::string matches_file,
    std::string cluster_path, std::string list_focal) {
    std::ifstream cluster;
    cluster.open(cluster_path);
    std::vector<int> correct_frame_id;
    int temp;
    while (cluster >> temp) {
      correct_frame_id.push_back(temp);
    }
    cluster.close();

    std::cerr << "Loaded cluster file\n";

    std::ifstream focalFile;
    focalFile.open(list_focal);
    std::vector<cv::Mat> all_images;
    std::string path, focal;
    while (focalFile >> path >> focal) {
      cv::Mat temp = cv::imread(data_dir + path);
      all_images.push_back(temp);
    }
    focalFile.close();

    std::cerr << "Loaded focal file and images\n";

    assert(all_images.size() == correct_frame_id.size());
    float cx = all_images[0].cols / 2;
    float cy = all_images[0].rows / 2;

    // std::ifstream matches;
    // matches.open(matches_file);
    FILE *fp;
    fp = fopen(matches_file.c_str(), "r");
    int num_pairs;
    fscanf(fp, "%d\n", &num_pairs);
    // matches >> num_pairs;
    for (int i = 0; i<num_pairs; i++) {
      int f1, f2, numc;
      fscanf(fp, "%d %d %d\n", &f1, &f2, &numc);
      // matches >> f1 >> f2 >> numc;
      assert(f1 < correct_frame_id.size());
      assert(f2 < correct_frame_id.size());
      for (int j = 0; j< numc; j++) {
        int sf1, sf2;
        float p1x, p1y, p2x, p2y;
        fscanf(fp, "%d %f %f %d %f %f\n", &sf1, &p1x, &p1y, &sf2, &p2x, &p2y);
        // matches >> sf1 >> p1x >> p1y >> sf2 >> p2x >> p2y;
        if (sf1 != sf2) {
          std::cerr << "Sift IDs did not match\n";
          assert(sf1 == sf2);
        }
        if (all_matches.find(sf1) == all_matches.end()) {
          all_matches[sf1] = world_corr();
          all_matches[sf1].location.x = 0;
          all_matches[sf1].location.y = 0;
          all_matches[sf1].location.z = 0;
          // Put in color
          all_matches[sf1].color = all_images[f1].at<cv::Vec3b>(cv::Point2f(p1x + cx, p1y + cy));
        }
        all_matches[sf1].pairs[correct_frame_id[f1]] = imgcorr(sf1, p1x, p1y);
        all_matches[sf2].pairs[correct_frame_id[f2]] = imgcorr(sf2, p2x, p2y);
      }
    }
    // matches.close();
    fclose(fp);
  }

  void writeWorld(std::string path) {
    std::ofstream output;
    output.open(path);
    for (auto it : all_matches) {
      output << it.second.location.x << " " << it.second.location.y << " " << it.second.location.z << " ";
      output << (int)it.second.color[2] << " " << (int)it.second.color[1] << " " << (int)it.second.color[0] << " ";
      output << it.second.pairs.size() << " ";
      for (auto it1 : it.second.pairs) {
        output << it1.first << " " << it1.second.siftid << " ";
        output << it1.second.imglocation.x << " " << it1.second.imglocation.y << " ";
      }
      output << "\n";
    }
    output.close();
  }
};

int main(int argc, char **argv)
{
  std::cout << "Entered corres\n";
  if (argc!=6) {
    std::cout << "Usage instructions: corres data_directory matches_file map_path list_focal_file world_path\n";
    return 1;
  }
  std::string FLAGS_data_dir(argv[1]);
  std::string FLAGS_matches_file(argv[2]);
  std::string FLAGS_map_path(argv[3]);
  std::string FLAGS_list_focal_file(argv[4]);
  std::string FLAGS_world_path(argv[5]);

  std::cerr << "Dir path: " << FLAGS_data_dir << "\n";
  std::cerr << "Matches file: " << FLAGS_matches_file << "\n";
  std::cerr << "Map Path: " << FLAGS_map_path << "\n";
  std::cerr << "List Focal File: " << FLAGS_list_focal_file << "\n";
  std::cerr << "World Path: " << FLAGS_world_path << "\n";

  total_world input(FLAGS_data_dir, FLAGS_matches_file, FLAGS_map_path, FLAGS_list_focal_file);
  std::cerr << "Done processing input\n";
  input.writeWorld(FLAGS_world_path);
  std::cout << "Leaving corres\n";
  return 0;
}
