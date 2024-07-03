#include <bbs3d/cpu_bbs3d/bbs3d.hpp>
#include <bbs3d/pointcloud_iof/pcl_eigen_converter.hpp>
#include <bbs3d/pointcloud_iof/pcd_loader.hpp>

#include <test.hpp>
#include <load_yaml.hpp>
#include <chrono>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

std::string create_date() {
  time_t t = time(nullptr);
  const tm* local_time = localtime(&t);
  std::stringstream s;
  s << "20" << local_time->tm_year - 100 << "_";
  s << local_time->tm_mon + 1 << "_";
  s << local_time->tm_mday << "_";
  s << local_time->tm_hour << "_";
  s << local_time->tm_min << "_";
  s << local_time->tm_sec;
  return (s.str());
}

BBS3DTest::BBS3DTest() {}

BBS3DTest::~BBS3DTest() {}

int BBS3DTest::run(std::string config) {
  // Load config file
  if (!load_config(config)) {
    std::cout << "[ERROR]  Loading config file failed" << std::endl;
    return 1;
  };

  // Load target pcds
  std::cout << "[Setting] Loading target pcds..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (!pciof::load_tar_clouds(tar_path, tar_leaf_size, tar_cloud_ptr)) {
    std::cout << "[ERROR] Loading target pcd failed" << std::endl;
    return 1;
  }

  std::unique_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_ptr;
  if (use_gicp) {
    gicp_ptr = std::make_unique<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    gicp_ptr->setInputTarget(tar_cloud_ptr);
  };

  // pcl to eigen
  std::vector<Eigen::Vector3d> tar_points;
  pciof::pcl_to_eigen(tar_cloud_ptr, tar_points);

  // Load source pcds
  std::cout << "[Setting] Loading source pcds..." << std::endl;
  std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> src_cloud_set;
  if (!pciof::load_src_points_with_filename(src_path, min_scan_range, max_scan_range, src_leaf_size, src_cloud_set)) {
    std::cout << "[ERROR] Loading source pcds failed" << std::endl;
    return 1;
  };

  // Create output folder with date
  std::cout << "[Setting] Create output folder with date..." << std::endl;
  std::string date = create_date();
  std::string pcd_save_folder_path = output_path + "/" + date;
  boost::filesystem::create_directory(pcd_save_folder_path);

  // ====3D-BBS====
  std::unique_ptr<cpu::BBS3D> bbs3d_ptr(new cpu::BBS3D);

  // Set target points
  std::cout << "[Voxel map] Creating hierarchical voxel map..." << std::endl;
  auto initi_t1 = std::chrono::high_resolution_clock::now();
  if (bbs3d_ptr->set_voxelmaps_coords(tar_path)) {
    std::cout << "[Voxel map] Loaded voxelmaps coords directly" << std::endl;
  } else {
    bbs3d_ptr->set_tar_points(tar_points, min_level_res, max_level);
    bbs3d_ptr->set_trans_search_range(tar_points);
  }
  auto init_t2 = std::chrono::high_resolution_clock::now();
  double init_time = std::chrono::duration_cast<std::chrono::nanoseconds>(init_t2 - initi_t1).count() / 1e6;
  std::cout << "[Voxel map] Execution time: " << init_time << "[msec] " << std::endl;

  bbs3d_ptr->set_angular_search_range(min_rpy, max_rpy);
  bbs3d_ptr->set_score_threshold_percentage(score_threshold_percentage);
  if (timeout_msec > 0) {
    bbs3d_ptr->enable_timeout();
    bbs3d_ptr->set_timeout_duration_in_msec(timeout_msec);
  }

  // num_threads
  int num_threads = 4;
  bbs3d_ptr->set_num_threads(num_threads);

  // localization
  double sum_time = 0;
  int num_localized = 0;
  for (const auto& src_cloud : src_cloud_set) {
    std::cout << "-------------------------------" << std::endl;
    std::cout << "[Localize] pcd file name: " << src_cloud.first << std::endl;
    std::vector<Eigen::Vector3d> src_points;
    pciof::pcl_to_eigen(src_cloud.second, src_points);
    bbs3d_ptr->set_src_points(src_points);
    bbs3d_ptr->localize();

    std::cout << "[Localize] Execution time: " << bbs3d_ptr->get_elapsed_time() << "[msec] " << std::endl;
    std::cout << "[Localize] Score: " << bbs3d_ptr->get_best_score() << std::endl;

    if (!bbs3d_ptr->has_localized()) {
      if (bbs3d_ptr->has_timed_out())
        std::cout << "[Failed] Localization timed out." << std::endl;
      else
        std::cout << "[Failed] Score is below the threshold." << std::endl;
      continue;
    }

    sum_time += bbs3d_ptr->get_elapsed_time();
    num_localized++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (use_gicp) {
      gicp_ptr->setInputSource(src_cloud.second);
      gicp_ptr->align(*output_cloud_ptr, bbs3d_ptr->get_global_pose().cast<float>());

      Eigen::Isometry3d estimated_pose;
      estimated_pose.matrix() = bbs3d_ptr->get_global_pose();
      Eigen::Isometry3d ground_truth_pose;
      ground_truth_pose.matrix() = gicp_ptr->final_transformation_.cast<double>();

      Eigen::Isometry3d error = estimated_pose.inverse() * ground_truth_pose;
      double error_t = error.translation().norm();
      double error_r = Eigen::AngleAxisd(error.linear()).angle();
      std::cout << "[Localize] Translation error: " << error_t << ", Rotation error: " << error_r << std::endl;
    } else {
      pcl::transformPointCloud(*src_cloud.second, *output_cloud_ptr, bbs3d_ptr->get_global_pose());
    }
    pcl::io::savePCDFileBinary(pcd_save_folder_path + "/" + src_cloud.first + ".pcd", *output_cloud_ptr);
  }

  if (num_localized != 0) {
    std::cout << "[Localize] Average time: " << sum_time / num_localized << "[msec] per frame" << std::endl;
  }

  return 0;
}

int main(int argc, char** argv) {
  std::string config = argv[1];
  BBS3DTest test;
  return test.run(config);
}
