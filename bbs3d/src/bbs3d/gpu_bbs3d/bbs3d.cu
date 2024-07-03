#include "bbs3d/gpu_bbs3d/bbs3d.cuh"
#include "bbs3d/gpu_bbs3d/stream_manager/check_error.cuh"
#include "bbs3d/discrete_transformation/discrete_transformation.hpp"

namespace gpu {
BBS3D::BBS3D()
: branch_copy_size_(10000),
  score_threshold_percentage_(0.0),
  use_timeout_(false),
  timeout_duration_(10000),
  has_timed_out_(false),
  has_localized_(false) {
  check_error << cudaStreamCreate(&stream);

  min_rpy_ << -0.02f, -0.02f, 0.0f;
  max_rpy_ << 0.02f, 0.02f, 2 * M_PI;
}

BBS3D::BBS3D(const DeviceVoxelMaps<float>::Ptr voxelmaps)
: voxelmaps_(voxelmaps),
  branch_copy_size_(10000),
  score_threshold_percentage_(0.0),
  use_timeout_(false),
  timeout_duration_(10000),
  has_timed_out_(false),
  has_localized_(false) {
  check_error << cudaStreamCreate(&stream);

  min_rpy_ << -0.02, -0.02, 0.0;
  max_rpy_ << 0.02, 0.02, 2 * M_PI;

  voxelmaps_->set_buckets_on_device(stream);
  set_trans_search_range_with_voxelmaps();
}

BBS3D::~BBS3D() {
  check_error << cudaStreamDestroy(stream);
}

void BBS3D::set_timeout_duration_in_msec(const int msec) {
  timeout_duration_ = std::chrono::milliseconds(msec);
}

// This function will be deprecated
bool BBS3D::set_voxelmaps_coords(const std::string& path) {
  voxelmaps_ = std::make_shared<DeviceVoxelMaps<float>>();
  if (!voxelmaps_->set_voxelmaps_coords(path)) return false;

  voxelmaps_->set_buckets_on_device(stream);
  set_trans_search_range_with_voxelmaps();
  return true;
}

void BBS3D::set_tar_points(const std::vector<Eigen::Vector3f>& points, float min_level_res, int max_level) {
  voxelmaps_ = std::make_shared<DeviceVoxelMaps<float>>(points, min_level_res, max_level);
}

void BBS3D::set_src_points(const std::vector<Eigen::Vector3f>& points) {
  int src_size = points.size();
  src_points_.clear();
  src_points_.shrink_to_fit();
  src_points_.resize(src_size);
  std::copy(points.begin(), points.end(), src_points_.begin());

  d_src_points_.clear();
  d_src_points_.shrink_to_fit();
  d_src_points_.resize(src_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points_.data()),
    points.data(),
    sizeof(Eigen::Vector3f) * src_size,
    cudaMemcpyHostToDevice,
    stream);
}

void BBS3D::set_trans_search_range(const std::vector<Eigen::Vector3f>& points) {
  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector3f min_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f max_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

  for (const auto& point : points) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  set_trans_search_range(min_xyz, max_xyz);
}

void BBS3D::set_trans_search_range(const Eigen::Vector3f& min_xyz, const Eigen::Vector3f& max_xyz) {
  min_xyz_ = min_xyz;
  max_xyz_ = max_xyz;

  const int max_level = voxelmaps_->max_level();
  const double top_res = voxelmaps_->info_vec_[max_level].res;
  init_tx_range_ = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
  init_ty_range_ = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
  init_tz_range_ = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
}

void BBS3D::set_trans_search_range_with_voxelmaps() {
  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector4i min_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::max());
  Eigen::Vector4i max_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::lowest());
  int max_level = voxelmaps_->max_level();
  const auto& top_buckets = voxelmaps_->buckets_vec_[max_level];
  for (const auto& bucket : top_buckets) {
    min_xyz = min_xyz.cwiseMin(bucket);
    max_xyz = max_xyz.cwiseMax(bucket);
  }
  init_tx_range_ = std::make_pair(min_xyz.x(), max_xyz.x());
  init_ty_range_ = std::make_pair(min_xyz.y(), max_xyz.y());
  init_tz_range_ = std::make_pair(min_xyz.z(), max_xyz.z());
}

void BBS3D::calc_angular_info(std::vector<AngularInfo>& ang_info_vec) {
  float max_norm = src_points_[0].norm();
  for (const auto& point : src_points_) {
    float norm = point.norm();
    if (norm > max_norm) {
      max_norm = norm;
    }
  }

  const int max_level = voxelmaps_->max_level();
  for (int i = max_level; i >= 0; i--) {
    const float cosine = 1 - (std::pow(voxelmaps_->info_vec_[i].res, 2) / std::pow(max_norm, 2)) * 0.5f;
    float ori_res = std::acos(max(cosine, -1.0f));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3f rpy_res_temp;
    rpy_res_temp.x() = ori_res <= (max_rpy_.x() - min_rpy_.x()) ? ori_res : 0.0f;
    rpy_res_temp.y() = ori_res <= (max_rpy_.y() - min_rpy_.y()) ? ori_res : 0.0f;
    rpy_res_temp.z() = ori_res <= (max_rpy_.z() - min_rpy_.z()) ? ori_res : 0.0f;

    Eigen::Vector3f max_rpy_piece;
    if (i == max_level) {
      max_rpy_piece = max_rpy_ - min_rpy_;
    } else {
      max_rpy_piece.x() = ang_info_vec[i + 1].rpy_res.x() != 0.0f ? ang_info_vec[i + 1].rpy_res.x() : max_rpy_.x() - min_rpy_.x();
      max_rpy_piece.y() = ang_info_vec[i + 1].rpy_res.y() != 0.0f ? ang_info_vec[i + 1].rpy_res.y() : max_rpy_.y() - min_rpy_.y();
      max_rpy_piece.z() = ang_info_vec[i + 1].rpy_res.z() != 0.0f ? ang_info_vec[i + 1].rpy_res.z() : max_rpy_.z() - min_rpy_.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() != 0.0f ? std::ceil(max_rpy_piece.x() / rpy_res_temp.x()) : 1;
    num_division.y() = rpy_res_temp.y() != 0.0f ? std::ceil(max_rpy_piece.y() / rpy_res_temp.y()) : 1;
    num_division.z() = rpy_res_temp.z() != 0.0f ? std::ceil(max_rpy_piece.z() / rpy_res_temp.z()) : 1;
    ang_info_vec[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec[i].rpy_res.x() = num_division.x() != 1 ? max_rpy_piece.x() / num_division.x() : 0.0f;
    ang_info_vec[i].rpy_res.y() = num_division.y() != 1 ? max_rpy_piece.y() / num_division.y() : 0.0f;
    ang_info_vec[i].rpy_res.z() = num_division.z() != 1 ? max_rpy_piece.z() / num_division.z() : 0.0f;

    ang_info_vec[i].min_rpy.x() = ang_info_vec[i].rpy_res.x() != 0.0f && ang_info_vec[i + 1].rpy_res.x() == 0.0f ? min_rpy_.x() : 0.0f;
    ang_info_vec[i].min_rpy.y() = ang_info_vec[i].rpy_res.y() != 0.0f && ang_info_vec[i + 1].rpy_res.y() == 0.0f ? min_rpy_.y() : 0.0f;
    ang_info_vec[i].min_rpy.z() = ang_info_vec[i].rpy_res.z() != 0.0f && ang_info_vec[i + 1].rpy_res.z() == 0.0f ? min_rpy_.z() : 0.0f;
  }
}

std::vector<DiscreteTransformation<float>> BBS3D::create_init_transset(const AngularInfo& init_ang_info) {
  const int init_transset_size = (init_tx_range_.second - init_tx_range_.first + 1) * (init_ty_range_.second - init_ty_range_.first + 1) *
                                 (init_tz_range_.second - init_tz_range_.first + 1) * (init_ang_info.num_division.x()) *
                                 (init_ang_info.num_division.y()) * (init_ang_info.num_division.z());

  const int max_level = voxelmaps_->max_level();

  std::vector<DiscreteTransformation<float>> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range_.first; tx <= init_tx_range_.second; tx++) {
    for (int ty = init_ty_range_.first; ty <= init_ty_range_.second; ty++) {
      for (int tz = init_tz_range_.first; tz <= init_tz_range_.second; tz++) {
        for (int roll = 0; roll < init_ang_info.num_division.x(); roll++) {
          for (int pitch = 0; pitch < init_ang_info.num_division.y(); pitch++) {
            for (int yaw = 0; yaw < init_ang_info.num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation<float>(0, max_level, tx, ty, tz, roll, pitch, yaw));
            }
          }
        }
      }
    }
  }
  return transset;
}

void BBS3D::localize() {
  // Calc localize time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + timeout_duration_;
  has_timed_out_ = false;

  // Initialize best score
  best_score_ = 0;
  const int score_threshold = std::floor(src_points_.size() * score_threshold_percentage_);
  int best_score = score_threshold;
  DiscreteTransformation<float> best_trans(best_score);

  // Preapre initial transset
  const int max_level = voxelmaps_->max_level();
  std::vector<AngularInfo> ang_info_vec(max_level + 1);
  calc_angular_info(ang_info_vec);
  thrust::device_vector<AngularInfo> d_ang_info_vec(ang_info_vec.size());
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_ang_info_vec.data()),
    ang_info_vec.data(),
    sizeof(AngularInfo) * ang_info_vec.size(),
    cudaMemcpyHostToDevice,
    stream);

  const auto init_transset = create_init_transset(ang_info_vec[max_level]);

  // Calc initial transset scores
  const auto init_transset_output = calc_scores(init_transset, d_ang_info_vec);

  std::priority_queue<DiscreteTransformation<float>> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation<float>> branch_stock;
  branch_stock.reserve(branch_copy_size_);
  while (!trans_queue.empty()) {
    if (use_timeout_ && std::chrono::system_clock::now() > time_limit) {
      has_timed_out_ = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // Calculate remaining branch_stock when queue is empty
    if (trans_queue.empty() && !branch_stock.empty()) {
      const auto transset_output = calc_scores(branch_stock, d_ang_info_vec);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }

    // pruning
    if (trans.score < best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      best_score = trans.score;
    } else {
      const int child_level = trans.level - 1;
      trans.branch(branch_stock, child_level, voxelmaps_->v_rate(), ang_info_vec[child_level].num_division);
    }

    if (branch_stock.size() >= branch_copy_size_) {
      const auto transset_output = calc_scores(branch_stock, d_ang_info_vec);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }
  }

  // Calc localization elapsed time
  const auto end_time = std::chrono::system_clock::now();
  elapsed_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

  // Not localized
  if (best_score == score_threshold || has_timed_out_) {
    has_localized_ = false;
    return;
  }

  global_pose_ = best_trans.create_matrix(voxelmaps_->min_res(), ang_info_vec[0].rpy_res, ang_info_vec[0].min_rpy);
  best_score_ = best_score;
  has_timed_out_ = false;
  has_localized_ = true;
}

__global__ void calc_scores_kernel(
  const thrust::device_ptr<Eigen::Vector4i*> multi_buckets_ptrs,
  const thrust::device_ptr<VoxelMapInfo<float>> voxelmap_info_ptr,
  const thrust::device_ptr<AngularInfo> d_ang_info_vec_ptr,
  thrust::device_ptr<DiscreteTransformation<float>> trans_ptr,
  size_t index_size,
  const thrust::device_ptr<Eigen::Vector3f> points_ptr,
  size_t num_points) {
  const size_t pose_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (pose_index > index_size) {
    return;
  }

  DiscreteTransformation<float>& trans = *thrust::raw_pointer_cast(trans_ptr + pose_index);
  const VoxelMapInfo<float>& voxelmap_info = *thrust::raw_pointer_cast(voxelmap_info_ptr + trans.level);
  const AngularInfo& ang_info = *thrust::raw_pointer_cast(d_ang_info_vec_ptr + trans.level);
  const Eigen::Vector4i* buckets = thrust::raw_pointer_cast(multi_buckets_ptrs)[trans.level];

  int score = 0;
  for (size_t i = 0; i < num_points; i++) {
    const Eigen::Vector3f& point = thrust::raw_pointer_cast(points_ptr)[i];

    const Eigen::Vector3f translation(trans.x * voxelmap_info.res, trans.y * voxelmap_info.res, trans.z * voxelmap_info.res);
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(trans.yaw * ang_info.rpy_res.z() + ang_info.min_rpy.z(), Eigen::Vector3f::UnitZ()) *
               Eigen::AngleAxisf(trans.pitch * ang_info.rpy_res.y() + ang_info.min_rpy.y(), Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(trans.roll * ang_info.rpy_res.x() + ang_info.min_rpy.x(), Eigen::Vector3f::UnitX());
    const Eigen::Vector3f transed_point = rotation * point + translation;

    // coord to hash
    const Eigen::Vector3i coord = (transed_point.array() * voxelmap_info.inv_res).floor().cast<int>();
    const std::uint32_t hash = (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);

    // open addressing
    for (int j = 0; j < voxelmap_info.max_bucket_scan_count; j++) {
      const std::uint32_t bucket_index = (hash + j) % voxelmap_info.num_buckets;
      const Eigen::Vector4i bucket = buckets[bucket_index];

      if (bucket.x() != coord.x() || bucket.y() != coord.y() || bucket.z() != coord.z()) {
        continue;
      }

      if (bucket.w() == 1) {
        score++;
        break;
      }
    }
  }
  trans.score = score;
}

std::vector<DiscreteTransformation<float>> BBS3D::calc_scores(
  const std::vector<DiscreteTransformation<float>>& h_transset,
  thrust::device_vector<AngularInfo>& d_ang_info_vec) {
  size_t transset_size = h_transset.size();
  thrust::device_vector<DiscreteTransformation<float>> d_transset(transset_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_transset.data()),
    h_transset.data(),
    sizeof(DiscreteTransformation<float>) * transset_size,
    cudaMemcpyHostToDevice,
    stream);

  const size_t block_size = 32;
  const size_t num_blocks = (transset_size + (block_size - 1)) / block_size;

  calc_scores_kernel<<<num_blocks, block_size, 0, stream>>>(
    voxelmaps_->d_buckets_ptrs_.data(),
    voxelmaps_->d_info_vec_.data(),
    d_ang_info_vec.data(),
    d_transset.data(),
    transset_size - 1,
    d_src_points_.data(),
    src_points_.size());

  std::vector<DiscreteTransformation<float>> h_output(transset_size);
  check_error << cudaMemcpyAsync(
    h_output.data(),
    thrust::raw_pointer_cast(d_transset.data()),
    sizeof(DiscreteTransformation<float>) * transset_size,
    cudaMemcpyDeviceToHost,
    stream);

  check_error << cudaStreamSynchronize(stream);
  return h_output;
}

}  // namespace gpu