#include <bbs3d/cpu_bbs3d/voxelmaps.hpp>

int main(int argc, char** argv) {
  std::string target_pcd_folder = argv[1];
  double min_res = std::stof(argv[2]);
  int max_level = std::stoi(argv[3]);

  bool overwrite = true;
  cpu::VoxelMaps<double> voxelmaps(target_pcd_folder, min_res, max_level, overwrite);

  return 0;
}