#include <bbs3d/gpu_bbs3d/bbs3d.cuh>
#include <bbs3d/pointcloud_iof/load.hpp>

// TODO Do this process with cpu version
int main(int argc, char** argv) {
  std::string target_pcd_folder = argv[1];
  float min_level_res = std::stof(argv[2]);
  int max_level = std::stoi(argv[3]);

  std::cout << "[Setting] Loading target pcds..." << std::endl;
  std::vector<Eigen::Vector3f> tar_points;
  float voxel_filter_width = 0.0f;
  if (!pciof::load_tar_clouds<float>(target_pcd_folder, voxel_filter_width, tar_points)) {
    std::cout << "[ERROR] Loading target pcd failed" << std::endl;
    return 1;
  }

  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  std::unique_ptr<gpu::BBS3D> bbs3d_ptr(new gpu::BBS3D);
  bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);

  std::cout << "[Voxel map] Saving voxel maps..." << std::endl;
  bbs3d_ptr->save_voxel_params(target_pcd_folder);
  bbs3d_ptr->save_voxelmaps_pcd(target_pcd_folder);
  return 0;
}