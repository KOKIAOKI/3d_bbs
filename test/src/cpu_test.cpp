#include <chrono>

// bbs3d
#include <bbs3d/cpu_bbs3d/bbs3d.hpp>
#include <bbs3d/pointcloud_iof/find_point_cloud_files.hpp>
#include <bbs3d/pointcloud_iof/pcl_eigen_converter.hpp>
#include <bbs3d/pointcloud_iof/pcl_loader.hpp>
#include <bbs3d/pointcloud_iof/filter.hpp>

// parameters and utils
#include "test_params.hpp"
#include "result_csv.hpp"
#include "util.h"

// pcl
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// small gicp
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
// #include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

int main(int argc, char** argv) {
  std::string config = argv[1];
  TestParams params(config);

  auto tar_cloud_files = pciof::find_point_cloud_files(params.tar_path);
  auto src_cloud_files = pciof::find_point_cloud_files(params.src_path);

  // sort src files
  if (pciof::can_convert_to_double(src_cloud_files)) {
    std::sort(src_cloud_files.begin(), src_cloud_files.end(), [](const std::string& a, const std::string& b) {
      return std::stod(std::filesystem::path(a).stem().string()) < std::stod(std::filesystem::path(b).stem().string());
    });
  }

  // merge point clouds and create target point cloud
  const auto pcl_tar_cloud = pciof::create_tar_cloud(tar_cloud_files);

  // gicp setting
  small_gicp::PointCloud::Ptr small_gicp_tar;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr tar_tree;
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> gicp;
  int vgicp_num_threads = 8;
  if (params.use_gicp) {
    small_gicp_tar = pcl_to_small_gicp(pcl_tar_cloud);
    small_gicp_tar = small_gicp::voxelgrid_sampling_omp(*small_gicp_tar, 0.0, vgicp_num_threads);
    tar_tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(small_gicp_tar, small_gicp::KdTreeBuilderOMP(vgicp_num_threads));
    small_gicp::estimate_covariances_omp(*small_gicp_tar, *tar_tree, 10, vgicp_num_threads);
    gicp.reduction.num_threads = vgicp_num_threads;
    gicp.rejector.max_dist_sq = 1.0;
  }

  // Create output folder with date
  std::string date = create_date();
  std::string pcd_save_folder_path = params.output_path + "/" + date;
  std::filesystem::create_directory(pcd_save_folder_path);
  ResultCsv result_csv(pcd_save_folder_path, params.gt_pose_csv_path);

  // ========================== 3D-BBS ==========================
  cpu::BBS3D bbs3d;
  bbs3d.score_threshold_percentage = params.score_threshold_percentage;
  bbs3d.num_threads = 4;
  bbs3d.use_timeout = params.timeout_duration_msec > 0;
  bbs3d.timeout_duration_msec = params.timeout_duration_msec;
  bbs3d.search_entire_map = params.search_entire_map;
  bbs3d.min_xyz = params.min_xyz;  // set search_entire_map to false
  bbs3d.max_xyz = params.max_xyz;  // set search_entire_map to false
  bbs3d.min_rpy = params.min_rpy;
  bbs3d.max_rpy = params.max_rpy;

  // Voxelmaps
  cpu::VoxelMaps<double> voxelmaps;
  bool has_set_voxelmaps = voxelmaps.set_voxelmaps(params.tar_path);
  if (!has_set_voxelmaps) {
    // TODO downsample in voxelmaps
    const auto tar_points = pciof::pcl_to_eigen<double>(pcl_tar_cloud);
    voxelmaps.create_voxelmaps(tar_points, params.min_level_res, params.max_level);
    voxelmaps.save_voxelmaps(params.tar_path);  // automatically save
  }

  // Localization
  double sum_time = 0;
  int num_localized = 0;
  for (const auto& src_cloud_file : src_cloud_files) {
    const auto pcl_src_cloud = pciof::load_point_cloud_file(src_cloud_file);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(params.src_leaf_size, params.src_leaf_size, params.src_leaf_size);
    filter.setInputCloud(pcl_src_cloud);
    filter.filter(*filtered_cloud_ptr);
    const auto src_points = pciof::pcl_to_eigen<double>(filtered_cloud_ptr);
    const auto cropped_src_points = pciof::narrow_scan_range<double>(src_points, params.min_scan_range, params.max_scan_range);

    std::cout << "-------------------------------" << std::endl;
    std::string file_name = std::filesystem::path(src_cloud_file).filename().string();
    std::cout << "[Localize] pcd file name: " << file_name << std::endl;

    const auto bbs3d_result = bbs3d.localize(voxelmaps, cropped_src_points);
    bbs3d_result.print();

    if (!bbs3d_result.localized) continue;

    sum_time += bbs3d_result.elapsed_time_msec;
    num_localized++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4d gicp_result_matrix = Eigen::Matrix4d::Identity();
    if (params.use_gicp) {
      auto small_gicp_src = pcl_to_small_gicp(pcl_src_cloud);
      small_gicp_src = small_gicp::voxelgrid_sampling_omp(*small_gicp_src, 0.2, vgicp_num_threads);
      auto src_tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(small_gicp_src, small_gicp::KdTreeBuilderOMP(vgicp_num_threads));
      small_gicp::estimate_covariances_omp(*small_gicp_src, *src_tree, 10, vgicp_num_threads);

      Eigen::Isometry3d bbs_T_target_source = Eigen::Isometry3d::Identity();
      bbs_T_target_source.matrix() = bbs3d_result.global_pose;
      const auto gicp_result = gicp.align(*small_gicp_tar, *small_gicp_src, *tar_tree, bbs_T_target_source);
      gicp_result_matrix = gicp_result.T_target_source.matrix();

      Eigen::Isometry3d error = bbs_T_target_source.inverse() * gicp_result.T_target_source;
      double error_t = error.translation().norm();
      double error_r = Eigen::AngleAxisd(error.linear()).angle();
      pcl::transformPointCloud(*pcl_src_cloud, *pcl_output_cloud, gicp_result_matrix.cast<float>());
    } else {
      pcl::transformPointCloud(*pcl_src_cloud, *pcl_output_cloud, bbs3d_result.global_pose.cast<float>());
    }
    pcl::io::savePCDFileBinary(pcd_save_folder_path + "/" + file_name + ".pcd", *pcl_output_cloud);
    result_csv
      .write(std::filesystem::path(src_cloud_file).stem().string(), bbs3d_result.elapsed_time_msec, bbs3d_result.global_pose, gicp_result_matrix);
  }

  if (num_localized != 0) std::cout << "[Localize] Average time: " << sum_time / num_localized << "[msec] per frame" << std::endl;

  return 0;
}
