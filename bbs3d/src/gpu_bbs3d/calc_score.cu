#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/voxelmaps.cuh>
#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {
__global__ void calc_scores_kernel(
  const thrust::device_ptr<Eigen::Vector4i*> multi_buckets_ptrs,
  const thrust::device_ptr<VoxelMapInfo> voxelmap_info_ptr,
  thrust::device_ptr<DiscreteTransformation> trans_ptr,
  size_t index_size,
  const thrust::device_ptr<Eigen::Vector3f> points_ptr,
  size_t num_points) {
  const size_t pose_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (pose_index > index_size) {
    return;
  }

  DiscreteTransformation& trans = *thrust::raw_pointer_cast(trans_ptr + pose_index);
  const VoxelMapInfo& voxelmap_info = *thrust::raw_pointer_cast(voxelmap_info_ptr + trans.level);
  const Eigen::Vector4i* buckets = thrust::raw_pointer_cast(multi_buckets_ptrs)[trans.level];

  int score = 0;
  for (size_t i = 0; i < num_points; i++) {
    const Eigen::Vector3f& point = thrust::raw_pointer_cast(points_ptr)[i];

    const Eigen::Vector3f translation(trans.x, trans.y, trans.z);
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX());
    const Eigen::Vector3f transed_point = rotation * point + translation;

    // coord to hash
    const Eigen::Vector3i coord = (transed_point.array() * voxelmap_info.inv_res).floor().cast<int>();
    const std::uint32_t hash = (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);

    for (int j = 0; j < voxelmap_info.max_bucket_scan_count; j++) {
      const std::uint32_t bucket_index = (hash + j) % voxelmap_info.num_buckets;
      const Eigen::Vector4i bucket = buckets[bucket_index];

      if (bucket.x() != coord.x() || bucket.y() != coord.y() || bucket.z() != coord.z()) {
        continue;
      }

      if (bucket.w() == 1) {
        score++;
      }
    }
  }
  trans.score = score;
}

std::vector<DiscreteTransformation> BBS3D::calc_scores(const std::vector<DiscreteTransformation>& h_transset) {
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
    voxelmaps_ptr_->d_multi_buckets_ptrs_.data(),
    voxelmaps_ptr_->d_voxelmaps_info_.data(),
    d_transset.data(),
    transset_size - 1,
    d_src_points_.data(),
    src_size_);

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