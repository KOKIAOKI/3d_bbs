#include <bbs3d/cpu_bbs3d/voxelmaps.hpp>
#include <bbs3d/pointcloud_iof/pcl_eigen_converter.hpp>
#include <bbs3d/pointcloud_iof/pcl_loader.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  std::string target_pcd_folder = argv[1];
  double min_res = std::stod(argv[2]);
  int max_level = std::stoi(argv[3]);

  const auto tar_cloud_files = pciof::find_point_cloud_files(target_pcd_folder);
  const auto pcl_tar_cloud = pciof::create_tar_cloud(tar_cloud_files);
  const auto target_points = pciof::pcl_to_eigen<double>(pcl_tar_cloud);

  cpu::VoxelMaps<double> voxelmaps;
  voxelmaps.create_voxelmaps(target_points, min_res, max_level);

  for (int level = max_level; level >= 0; level--) {
    std::cout << "Voxelmap level: " << level << std::endl;
    auto viewer = guik::viewer();
    size_t cnt = 0;
    const auto& buckets = voxelmaps.buckets_vec[level];
    for (const auto& bucket : buckets) {
      Eigen::Vector3f coord = bucket.cast<float>().head<3>();
      auto transformation = Eigen::Translation3f(coord.x(), coord.y(), coord.z());
      Eigen::Vector4f color(1.0f, 1.0f, 1.0f, 0.5f);
      auto setting1 = guik::FlatColor(color, transformation);
      viewer->update_drawable(std::to_string(cnt), glk::Primitives::cube(), setting1);

      cnt++;
    }
    viewer->spin();  // To show the next level voxelmap, close the window
  }
  return 0;
}