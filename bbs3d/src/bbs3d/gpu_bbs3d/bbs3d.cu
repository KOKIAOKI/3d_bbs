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

  // Calc BBS time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + std::chrono::milliseconds(timeout_duration_msec);

  // Score threshold
  const int score_threshold = std::floor(src_points_size * score_threshold_percentage);
  DiscreteTransformation<float> best_trans(score_threshold);
  // Calc angular info
  if (calc_ang_info) {
    const auto max_norm = calc_max_norm(src_points);
    d_voxelmaps_->calc_angular_info(max_norm, min_rpy, max_rpy, stream);
  }

  // copy host src_points to device
  thrust::device_vector<Eigen::Vector3f> d_src_points;
  d_src_points.resize(src_points_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points.data()),
    src_points.data(),
    sizeof(Eigen::Vector3f) * src_points_size,
    cudaMemcpyHostToDevice,
    stream);
  check_error << cudaStreamSynchronize(stream);

  // Preapre initial transset
  auto init_transset = create_init_transset();
  const auto init_transset_output = calc_scores(init_transset, d_src_points, src_points_size);
  std::priority_queue<DiscreteTransformation<float>> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation<float>> branch_stock;
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
      trans.branch(branch_stock, child_level, d_voxelmaps_->h_ang_info_vec[child_level].num_division);
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

  result.global_pose = best_trans.create_matrix(d_voxelmaps_->pose_to_matrix_tool(0));
  result.localized = true;
  result.timed_out = false;

  return result;
}

float BBS3D::calc_max_norm(const std::vector<Eigen::Vector3f>& src_points) {
  float max_norm = src_points[0].norm();
  for (const auto& point : src_points) {
    float norm = point.norm();
    if (norm > max_norm) {
      max_norm = norm;
    }
  }
  return max_norm;
}

std::vector<DiscreteTransformation<float>> BBS3D::create_init_transset() {
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
  const auto& ang_num_division = d_voxelmaps_->h_ang_info_vec[max_level].num_division;

  const int init_transset_size = (init_tx_range.second - init_tx_range.first + 1) * (init_ty_range.second - init_ty_range.first + 1) *
                                 (init_tz_range.second - init_tz_range.first + 1) * (ang_num_division.x()) * (ang_num_division.y()) *
                                 (ang_num_division.z());

  std::vector<DiscreteTransformation<float>> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range.first; tx <= init_tx_range.second; tx++) {
    for (int ty = init_ty_range.first; ty <= init_ty_range.second; ty++) {
      for (int tz = init_tz_range.first; tz <= init_tz_range.second; tz++) {
        for (int roll = 0; roll < ang_num_division.x(); roll++) {
          for (int pitch = 0; pitch < ang_num_division.y(); pitch++) {
            for (int yaw = 0; yaw < ang_num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation<float>(0, max_level, tx, ty, tz, roll, pitch, yaw));
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
  thrust::device_ptr<DiscreteTransformation<float>> trans_ptr,
  size_t index_size,
  const thrust::device_ptr<const Eigen::Vector3f> points_ptr,
  size_t num_points) {
  const size_t pose_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (pose_index > index_size) {
    return;
  }

  DiscreteTransformation<float>& trans = *thrust::raw_pointer_cast(trans_ptr + pose_index);
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

std::vector<DiscreteTransformation<float>> BBS3D::calc_scores(
  const std::vector<DiscreteTransformation<float>>& h_transset,
  const thrust::device_vector<Eigen::Vector3f>& d_src_points,
  const size_t src_points_size) {
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
    d_voxelmaps_->d_buckets_ptrs.data(),
    d_voxelmaps_->d_info_vec.data(),
    d_voxelmaps_->d_ang_info_vec.data(),
    d_transset.data(),
    transset_size - 1,
    d_src_points.data(),
    src_points_size);  // TODO

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