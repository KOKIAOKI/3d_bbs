#include "bbs3d/gpu_bbs3d/voxelmaps.cuh"

namespace gpu {

DeviceVoxelMaps::DeviceVoxelMaps(const cpu::VoxelMaps<float>& voxelmaps, cudaStream_t stream)
: min_res_(voxelmaps.min_res()),
  max_res_(voxelmaps.max_res()),
  max_level_(voxelmaps.max_level()),
  top_tx_range_(voxelmaps.top_tx_range()),
  top_ty_range_(voxelmaps.top_ty_range()),
  top_tz_range_(voxelmaps.top_tz_range()) {
  copy_buckets_to_device(voxelmaps.buckets_vec, stream);
  copy_voxel_info_to_device(voxelmaps.info_vec, stream);
  check_error << cudaStreamSynchronize(stream);
}

void DeviceVoxelMaps::copy_buckets_to_device(const std::vector<Buckets>& buckets_vec, cudaStream_t stream) {
  d_buckets_vec.resize(buckets_vec.size());

  for (int i = 0; i < buckets_vec.size(); i++) {
    const int buckets_size = buckets_vec[i].size();
    const int buckets_datasize = sizeof(Eigen::Vector4i) * buckets_size;
    DeviceBuckets d_buckets(buckets_size);
    check_error
      << cudaMemcpyAsync(thrust::raw_pointer_cast(d_buckets.data()), buckets_vec[i].data(), buckets_datasize, cudaMemcpyHostToDevice, stream);
    d_buckets_vec[i] = d_buckets;
  }

  // Extract d_buckets_vec pointers
  std::vector<Eigen::Vector4i*> ptrs;
  ptrs.reserve(buckets_vec.size());
  for (int i = 0; i < buckets_vec.size(); i++) {
    ptrs.emplace_back(thrust::raw_pointer_cast(d_buckets_vec[i].data()));
  }

  // Copy host to device (ptrs)
  d_buckets_ptrs.resize(buckets_vec.size());
  const int ptrs_datasize = sizeof(Eigen::Vector4i*) * buckets_vec.size();
  check_error << cudaMemcpyAsync(thrust::raw_pointer_cast(d_buckets_ptrs.data()), ptrs.data(), ptrs_datasize, cudaMemcpyHostToDevice, stream);
}

void DeviceVoxelMaps::copy_voxel_info_to_device(const std::vector<cpu::VoxelMapInfo<float>>& cpu_info_vec, cudaStream_t stream) {
  // copy info vec
  info_vec.resize(cpu_info_vec.size());
  for (int i = 0; i < info_vec.size(); i++) {
    info_vec[i].num_buckets = cpu_info_vec[i].num_buckets;
    info_vec[i].max_bucket_scan_count = cpu_info_vec[i].max_bucket_scan_count;
    info_vec[i].res = cpu_info_vec[i].res;
    info_vec[i].inv_res = cpu_info_vec[i].inv_res;
  }

  // Copy host to device (voxel map info)
  d_info_vec.resize(info_vec.size());
  const int info_datasize = sizeof(VoxelMapInfo) * info_vec.size();
  check_error << cudaMemcpyAsync(thrust::raw_pointer_cast(d_info_vec.data()), info_vec.data(), info_datasize, cudaMemcpyHostToDevice, stream);
}
}  // namespace gpu
