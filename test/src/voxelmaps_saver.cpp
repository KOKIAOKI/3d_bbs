#include <cpu_bbs3d/bbs3d.hpp>
#include <pointcloud_iof/pcd_loader_without_pcl.hpp>

#include <algorithm>
#include <iterator>

int main(int argc, char** argv) {
  std::string target_pcd_folder = argv[1];
  double min_level_res = std::stof(argv[2]);
  int max_level = std::stoi(argv[3]);

  std::cout << "[Setting] Loading target pcds..." << std::endl;
  std::vector<Eigen::Vector3f> tar_points;
  float voxel_filter_width = 0.0f;
  if (!pciof::load_tar_points<float>(target_pcd_folder, voxel_filter_width, tar_points)) {
    std::cout << "[ERROR] Loading target pcd failed" << std::endl;
    return 1;
  }

  std::vector<Eigen::Vector3d> tar_points_d;
  tar_points_d.resize(tar_points.size());
  std::transform(tar_points.begin(), tar_points.end(), tar_points_d.begin(), [](const Eigen::Vector3f& point) { return point.cast<double>(); });

  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  std::unique_ptr<cpu::BBS3D> bbs3d_ptr(new cpu::BBS3D);
  bbs3d_ptr->set_tar_points(tar_points_d, min_level_res, max_level);

  std::cout << "[Voxel map] Saving voxel maps..." << std::endl;
  bbs3d_ptr->save_voxel_params(target_pcd_folder);
  bbs3d_ptr->save_voxelmaps_pcd(target_pcd_folder);
  return 0;
}