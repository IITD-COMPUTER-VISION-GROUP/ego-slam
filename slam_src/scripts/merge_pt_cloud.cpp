#include <iostream>
#include "nvmhelpers.h"
#include "intermediate_helpers.h"
#include <gflags/gflags.h>
#include <string>

DEFINE_string(path1, "btp/batch0/", "Path to dir of 1st nvm file");
DEFINE_string(path2, "btp/batch1/", "Path to dir of 2nd nvm file");
DEFINE_string(path_data, "data2/", "Path where lst focal global is present");
DEFINE_string(output_dir, "btp/", "Output combined ply file");

std::vector<std::string> read_focal_file(std::string path) {
  std::ifstream inp;
  inp.open(path);
  std::vector<std::string> answer;
  std::string temp, t2;
  while (inp >> temp >> t2) {
    answer.push_back(temp);
  }
  inp.close();
  return answer;
}

int main(int argc, char **argv)
{
  gflags::SetUsageMessage("dense --help");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  nvm_file f1(FLAGS_path1 + "outputVSFM_GB.nvm");
  std::cerr << "Done reading 1\n";
  nvm_file f2(FLAGS_path2 + "outputVSFM_GB.nvm");
  std::cerr << "Done reading 2\n";
  intermediate_file inter(FLAGS_path1 + "IntermediateRT.txt");
  std::vector<std::string> list_focal_global = read_focal_file(FLAGS_path_data + "list_focal.txt");

  GetBestRST(f1, f2);

  nvm_file merged = merge_nvm(f1, f2);
  // add_intermediate_points(merged, inter, list_focal_global);

  merged.save_to_disk(FLAGS_output_dir + "combined.nvm");
  merged.save_ply_file(FLAGS_output_dir + "combined.ply");

  merged.save_focal_for_optimisation(FLAGS_output_dir + "focal_for_optimisation.txt");
  merged.save_rt_global_file(FLAGS_output_dir + "RTglobalmapped_for_optimisation.txt");
  merged.save_distortion_file(FLAGS_output_dir + "Distortion.txt");
  merged.save_ours_new(FLAGS_output_dir+"ours_new.txt");
  merged.save_invmap(FLAGS_output_dir+"Invmap.txt");

  return 0;
}
