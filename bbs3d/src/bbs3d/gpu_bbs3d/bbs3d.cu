#include "bbs3d/gpu_bbs3d/bbs3d.cuh"
#include "bbs3d/gpu_bbs3d/stream_manager/check_error.cuh"
#include "bbs3d/discrete_transformation/discrete_transformation.hpp"

namespace gpu {
BBS3D::BBS3D() {
  check_error << cudaStreamCreate(&stream);
}

BBS3D::~BBS3D() {
  check_error << cudaStreamDestroy(stream);
}

void BBS3D::copy_voxelmaps_to_device(const cpu::VoxelMaps<float>& voxelmaps) {
  d_voxelmaps_ = std::make_shared<DeviceVoxelMaps>(voxelmaps, stream);
}

BBSResult BBS3D::localize(const std::vector<Eigen::Vector3f>& src_points) {
  BBSResult result;
  size_t src_points_size = src_points.size();

  // Score threshold
  const int score_threshold = std::floor(src_points_size * score_threshold_percentage);
  DiscreteTransformation best_trans(score_threshold);
  // Calc angular info
  if (calc_ang_info) {
    auto max_norm_iter =
      std::max_element(src_points.begin(), src_points.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return a.norm() < b.norm(); });
    calc_angular_info(max_norm_iter->norm());
  }

  // Copy host src_points to device
  thrust::device_vector<Eigen::Vector3f> d_src_points;
  d_src_points.resize(src_points_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points.data()),
    src_points.data(),
    sizeof(Eigen::Vector3f) * src_points_size,
    cudaMemcpyHostToDevice,
    stream);
  check_error << cudaStreamSynchronize(stream);

  // Calc BBS time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + std::chrono::milliseconds(timeout_duration_msec);

  // Preapre initial transset
  auto init_transset = create_init_transset();
  const auto init_transset_output = calc_scores(init_transset, d_src_points, src_points_size);
  std::priority_queue<DiscreteTransformation> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation> branch_stock;
  branch_stock.reserve(branch_copy_size);
  while (!trans_queue.empty()) {
    if (use_timeout && std::chrono::system_clock::now() > time_limit) {
      result.timed_out = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // Calculate remaining branch_stock when queue is empty
    if (trans_queue.empty() && !branch_stock.empty()) {
      const auto transset_output = calc_scores(branch_stock, d_src_points, src_points_size);
      for (const auto& output : transset_output) {
        if (output.score < result.best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }

    // pruning
    if (trans.score < result.best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      result.best_score = trans.score;
    } else {
      const int child_level = trans.level - 1;
      trans.branch(branch_stock, child_level, ang_info_vec_[child_level].num_division);
    }

    if (branch_stock.size() >= branch_copy_size) {
      const auto transset_output = calc_scores(branch_stock, d_src_points, src_points_size);
      for (const auto& output : transset_output) {
        if (output.score < result.best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }
  }

  // Calc localization elapsed time
  const auto end_time = std::chrono::system_clock::now();
  result.elapsed_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

  // Not localized
  if (result.best_score == score_threshold || result.timed_out) {
    result.localized = false;
    return result;
  }

  result.global_pose = best_trans.create_matrix(d_voxelmaps_->info_vec[0].res, ang_info_vec_[0].rpy_res, ang_info_vec_[0].min_rpy);
  result.localized = true;
  result.timed_out = false;

  return result;
}

void BBS3D::calc_angular_info(const float max_norm) {
  const size_t max_level = d_voxelmaps_->max_level();
  ang_info_vec_.clear();
  ang_info_vec_.resize(max_level + 1);
  for (int i = max_level; i >= 0; i--) {
    const float cosine = 1 - (std::pow(d_voxelmaps_->info_vec[i].res, 2) / std::pow(max_norm, 2)) * 0.5;
    float ori_res = std::acos(std::max(cosine, static_cast<float>(-1.0)));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3f rpy_res_temp;
    rpy_res_temp.x() = ori_res <= std::abs(max_rpy.x() - min_rpy.x()) ? ori_res : 0.0;
    rpy_res_temp.y() = ori_res <= std::abs(max_rpy.y() - min_rpy.y()) ? ori_res : 0.0;
    rpy_res_temp.z() = ori_res <= std::abs(max_rpy.z() - min_rpy.z()) ? ori_res : 0.0;

    Eigen::Vector3f max_rpypiece;
    if (i == max_level) {
      max_rpypiece = max_rpy - min_rpy;
    } else {
      max_rpypiece.x() = ang_info_vec_[i + 1].rpy_res.x() != 0.0 ? ang_info_vec_[i + 1].rpy_res.x() : max_rpy.x() - min_rpy.x();
      max_rpypiece.y() = ang_info_vec_[i + 1].rpy_res.y() != 0.0 ? ang_info_vec_[i + 1].rpy_res.y() : max_rpy.y() - min_rpy.y();
      max_rpypiece.z() = ang_info_vec_[i + 1].rpy_res.z() != 0.0 ? ang_info_vec_[i + 1].rpy_res.z() : max_rpy.z() - min_rpy.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() == 0.0 ? 1 : std::ceil(max_rpypiece.x() / rpy_res_temp.x());
    num_division.y() = rpy_res_temp.y() == 0.0 ? 1 : std::ceil(max_rpypiece.y() / rpy_res_temp.y());
    num_division.z() = rpy_res_temp.z() == 0.0 ? 1 : std::ceil(max_rpypiece.z() / rpy_res_temp.z());
    ang_info_vec_[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec_[i].rpy_res.x() = num_division.x() == 1 ? 0.0 : max_rpypiece.x() / num_division.x();
    ang_info_vec_[i].rpy_res.y() = num_division.y() == 1 ? 0.0 : max_rpypiece.y() / num_division.y();
    ang_info_vec_[i].rpy_res.z() = num_division.z() == 1 ? 0.0 : max_rpypiece.z() / num_division.z();

    ang_info_vec_[i].min_rpy.x() = ang_info_vec_[i].rpy_res.x() == 0.0 ? 0.0 : min_rpy.x();
    ang_info_vec_[i].min_rpy.y() = ang_info_vec_[i].rpy_res.y() == 0.0 ? 0.0 : min_rpy.y();
    ang_info_vec_[i].min_rpy.z() = ang_info_vec_[i].rpy_res.z() == 0.0 ? 0.0 : min_rpy.z();
  }

  d_ang_info_vec_.clear();
  d_ang_info_vec_.shrink_to_fit();
  d_ang_info_vec_.resize(max_level + 1);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_ang_info_vec_.data()),
    ang_info_vec_.data(),
    sizeof(AngularInfo) * ang_info_vec_.size(),
    cudaMemcpyHostToDevice,
    stream);
}

std::vector<DiscreteTransformation> BBS3D::create_init_transset() {
  std::pair<int, int> init_tx_range, init_ty_range, init_tz_range;
  if (search_entire_map) {
    init_tx_range = d_voxelmaps_->top_tx_range();
    init_ty_range = d_voxelmaps_->top_ty_range();
    init_tz_range = d_voxelmaps_->top_tz_range();
  } else {
    float top_res = d_voxelmaps_->max_res();
    init_tx_range = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
    init_ty_range = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
    init_tz_range = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
  }

  const int max_level = d_voxelmaps_->max_level();
  const auto& ang_num_division = ang_info_vec_[max_level].num_division;

  const int init_transset_size = (init_tx_range.second - init_tx_range.first + 1) * (init_ty_range.second - init_ty_range.first + 1) *
                                 (init_tz_range.second - init_tz_range.first + 1) * (ang_num_division.x()) * (ang_num_division.y()) *
                                 (ang_num_division.z());

  std::vector<DiscreteTransformation> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range.first; tx <= init_tx_range.second; tx++) {
    for (int ty = init_ty_range.first; ty <= init_ty_range.second; ty++) {
      for (int tz = init_tz_range.first; tz <= init_tz_range.second; tz++) {
        for (int roll = 0; roll < ang_num_division.x(); roll++) {
          for (int pitch = 0; pitch < ang_num_division.y(); pitch++) {
            for (int yaw = 0; yaw < ang_num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation(0, max_level, tx, ty, tz, roll, pitch, yaw));
            }
          }
        }
      }
    }
  }
  return transset;
}

__global__ void calc_scores_kernel(
  const thrust::device_ptr<Eigen::Vector4i*> multi_buckets_ptrs,
  const thrust::device_ptr<VoxelMapInfo> voxelmap_info_ptr,
  const thrust::device_ptr<AngularInfo> d_ang_info_vec_ptr,
  thrust::device_ptr<DiscreteTransformation> trans_ptr,
  size_t index_size,
  const thrust::device_ptr<const Eigen::Vector3f> points_ptr,
  size_t num_points) {
  const size_t pose_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (pose_index > index_size) {
    return;
  }

  DiscreteTransformation& trans = *thrust::raw_pointer_cast(trans_ptr + pose_index);
  const VoxelMapInfo& voxelmap_info = *thrust::raw_pointer_cast(voxelmap_info_ptr + trans.level);
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

std::vector<DiscreteTransformation> BBS3D::calc_scores(
  const std::vector<DiscreteTransformation>& h_transset,
  const thrust::device_vector<Eigen::Vector3f>& d_src_points,
  const size_t src_points_size) {
  size_t transset_size = h_transset.size();

  thrust::device_vector<DiscreteTransformation> d_transset(transset_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_transset.data()),
    h_transset.data(),
    sizeof(DiscreteTransformation) * transset_size,
    cudaMemcpyHostToDevice,
    stream);

  const size_t block_size = 32;
  const size_t num_blocks = (transset_size + (block_size - 1)) / block_size;

  calc_scores_kernel<<<num_blocks, block_size, 0, stream>>>(
    d_voxelmaps_->d_buckets_ptrs.data(),
    d_voxelmaps_->d_info_vec.data(),
    d_ang_info_vec_.data(),
    d_transset.data(),
    transset_size - 1,
    d_src_points.data(),
    src_points_size);

  std::vector<DiscreteTransformation> h_output(transset_size);
  check_error << cudaMemcpyAsync(
    h_output.data(),
    thrust::raw_pointer_cast(d_transset.data()),
    sizeof(DiscreteTransformation) * transset_size,
    cudaMemcpyDeviceToHost,
    stream);

  check_error << cudaStreamSynchronize(stream);
  return h_output;
}

}  // namespace gpu