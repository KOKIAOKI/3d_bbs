#include <gpu_bbs3d/bbs3d.cuh>
#include <test.hpp>
#include <util.hpp>
#include <load.hpp>
#include <chrono>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

BBS3DTest::BBS3DTest() {}

BBS3DTest::~BBS3DTest() {}

int BBS3DTest::run(std::string config) {
  // Load config file
  if (!load_config(config)) {
    std::cout << "[ERROR] load config file" << std::endl;
    return 1;
  };

  if (use_gicp) gicp_ptr.reset(new GICP);

  // Load target pcds
  std::vector<Eigen::Vector3f> tar_points;
  if (!load_tar_clouds(tar_points)) {
    std::cout << "[ERROR] load target clouds" << std::endl;
    return 1;
  }

  // Load source pcds
  std::cout << "Load source pcds" << std::endl;
  std::map<std::string, std::vector<Eigen::Vector3f>> src_points_set;
  if (!load_src_clouds(src_points_set)) {
    std::cout << "[ERROR] load source clouds" << std::endl;
    return 1;
  };

  // Create output folder with date
  std::cout << "Create output folder with date" << std::endl;
  std::string date = createDate();
  std::string pcd_save_folder_path = output_path + "/" + date;
  boost::filesystem::create_directory(pcd_save_folder_path);

  // ====3D-BBS====
  // Set target points
  auto initi_t1 = std::chrono::high_resolution_clock::now();
  std::cout << "Set target points" << std::endl;
  std::unique_ptr<gpu::BBS3D> bbs3d_ptr(new gpu::BBS3D);
  bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);
  bbs3d_ptr->set_angular_search_range(min_rpy.cast<float>(), max_rpy.cast<float>());

  auto init_t2 = std::chrono::high_resolution_clock::now();
  double init_time = std::chrono::duration_cast<std::chrono::nanoseconds>(init_t2 - initi_t1).count() / 1e6;
  std::cout << "[Hierarchical voxel map creation]Execution time: " << init_time << "[msec] " << std::endl;

  int sum_time = 0;
  // localization
  for (const auto& src_points : src_points_set) {
    std::cout << "-------------------------------" << std::endl;
    bbs3d_ptr->set_src_points(src_points.second);

    auto localize_t1 = std::chrono::high_resolution_clock::now();
    bbs3d_ptr->set_score_threshold_percentage(static_cast<float>(score_threshold_percentage));
    bbs3d_ptr->localize();
    auto localize_t2 = std::chrono::high_resolution_clock::now();
    double localize_time = std::chrono::duration_cast<std::chrono::nanoseconds>(localize_t2 - localize_t1).count() / 1e6;
    std::cout << "[Localize]Execution time: " << localize_time << "[msec] " << std::endl;
    std::cout << "[Localize]Score: " << bbs3d_ptr->get_best_score() << std::endl;

    if (!bbs3d_ptr->has_localized()) {
      std::cout << "[Localize]Failed: score is below the threshold." << std::endl;
      continue;
    }

    sum_time += localize_time;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    eigen_to_pcl(src_points.second, src_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (use_gicp) {
      gicp_ptr->setInputSource(src_cloud_ptr);
      gicp_ptr->align(*output_cloud_ptr, bbs3d_ptr->get_global_pose());
    } else {
      pcl::transformPointCloud(*src_cloud_ptr, *output_cloud_ptr, bbs3d_ptr->get_global_pose());
    }
    pcl::io::savePCDFileBinary(pcd_save_folder_path + "/" + src_points.first + ".pcd", *output_cloud_ptr);
  }
  std::cout << "[Localize]Average time: " << sum_time / src_points_set.size() << "[msec] per frame" << std::endl;
  return 0;
}

int main(int argc, char** argv) {
  std::string config = argv[1];
  BBS3DTest test;
  return test.run(config);
}