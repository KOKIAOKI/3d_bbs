#include <bbs3d/cpu_bbs3d/voxelmaps.hpp>
#include <bbs3d/pointcloud_iof/pcl_eigen_converter.hpp>
#include <bbs3d/pointcloud_iof/pcl_loader.hpp>

int main(int argc, char** argv) {
  std::string target_pcd_folder = argv[1];
  double min_res = std::stod(argv[2]);
  int max_level = std::stoi(argv[3]);

  const auto tar_cloud_files = pciof::find_point_cloud_files(target_pcd_folder);
  const auto pcl_tar_cloud = pciof::create_tar_cloud(tar_cloud_files);
  const auto target_points = pciof::pcl_to_eigen<double>(pcl_tar_cloud);

  cpu::VoxelMaps<double> voxelmaps;
  voxelmaps.create_voxelmaps(target_points, min_res, max_level);
  if (!voxelmaps.save_voxelmaps(target_pcd_folder)) {
    std::cerr << "Failed to save voxelmap files." << std::endl;
    return 1;
  }

  std::cout << "Voxelmap files were successfully saved." << std::endl;
  return 0;
}