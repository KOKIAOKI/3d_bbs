#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>

// thrust
#include <cuda_runtime.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace gpu {
class VoxelMaps;

struct AngularInfo {
  Eigen::Vector3i num_division;
  Eigen::Vector3f rpy_res;
  Eigen::Vector3f min_rpy;
};

struct DiscreteTransformation {
public:
  DiscreteTransformation();
  DiscreteTransformation(int score);
  DiscreteTransformation(int score, int level, float resolution, float x, float y, float z, float roll, float pitch, float yaw);
  ~DiscreteTransformation();

  bool operator<(const DiscreteTransformation& rhs) const;

  bool is_leaf() const;

  Eigen::Matrix4f create_matrix();

  void branch(std::vector<DiscreteTransformation>& b, const int child_level, const float child_res, const int v_rate, const AngularInfo& ang_info);

public:
  int score;
  int level;
  float resolution;
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

  class BBS3D {
  public:
    BBS3D();
    ~BBS3D();

    void set_tar_points(const std::vector<Eigen::Vector3f>& points, float min_level_res, int max_level);

    void set_src_points(const std::vector<Eigen::Vector3f>& points);

    void set_trans_search_range(const std::vector<Eigen::Vector3f>& points);

    void set_angular_search_range(const Eigen::Vector3f& min_rpy, const Eigen::Vector3f& max_rpy) {
      min_rpy_ = min_rpy;
      max_rpy_ = max_rpy;
    }

    void set_voxel_expantion_rate(const float rate) {
      v_rate_ = rate;
      inv_v_rate_ = 1.0f / rate;
    }

    void set_branch_copy_size(int size) { branch_copy_size_ = size; }

    void set_score_threshold_percentage(float percentage) { score_threshold_percentage_ = percentage; }

    std::vector<Eigen::Vector3f> get_src_points() const { return src_points_; }

    std::vector<Eigen::Vector3f> get_tar_points() const { return tar_points_; }

    std::vector<std::pair<int, int>> get_trans_search_range() const {
      return std::vector<std::pair<int, int>>{init_tx_range_, init_ty_range_, init_tz_range_};
    }

    std::vector<Eigen::Vector3f> get_angular_search_range() const { return std::vector<Eigen::Vector3f>{min_rpy_, max_rpy_}; }

    Eigen::Matrix4f get_global_pose() const { return global_pose_; }

    int get_best_score() const { return best_score_; }

    bool has_localized() { return has_localized_; }

    void localize();

  private:
    void calc_angluar_info(std::vector<AngularInfo>& ang_info_vec);

    std::vector<DiscreteTransformation> create_init_transset(const AngularInfo& init_ang_info);

    std::vector<DiscreteTransformation> calc_scores(const std::vector<DiscreteTransformation>& h_transset);

  private:
    Eigen::Matrix4f global_pose_;
    bool has_localized_;

    cudaStream_t stream;

    std::vector<Eigen::Vector3f> tar_points_;
    std::vector<Eigen::Vector3f> src_points_;

    thrust::device_vector<Eigen::Vector3f> d_src_points_;
    std::vector<thrust::device_vector<DiscreteTransformation>> d_transset_stock_;

    std::unique_ptr<VoxelMaps> voxelmaps_ptr_;

    float v_rate_;  // voxel expansion rate
    float inv_v_rate_;

    int src_size_, branch_copy_size_, best_score_;
    double score_threshold_percentage_;
    Eigen::Vector3f min_rpy_;
    Eigen::Vector3f max_rpy_;
    std::pair<int, int> init_tx_range_;
    std::pair<int, int> init_ty_range_;
    std::pair<int, int> init_tz_range_;
  };
  }  // namespace gpu