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

void DeviceVoxelMaps::copy_voxel_info_to_device(const std::vector<cpu::VoxelMapInfo<float>>& info_vec, cudaStream_t stream) {
  h_info_vec = info_vec;
  d_info_vec.resize(info_vec.size());

  // Copy host to device (voxel map info)
  d_info_vec.resize(info_vec.size());
  const int info_datasize = sizeof(VoxelMapInfo) * info_vec.size();
  check_error << cudaMemcpyAsync(thrust::raw_pointer_cast(d_info_vec.data()), info_vec.data(), info_datasize, cudaMemcpyHostToDevice, stream);
}

void DeviceVoxelMaps::copy_ang_info_to_device(const std::vector<AngularInfo>& ang_info_vec, cudaStream_t stream) {
  // Preapre initial transset
  h_ang_info_vec = ang_info_vec;
  d_ang_info_vec.resize(max_level_ + 1);

  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_ang_info_vec.data()),
    h_ang_info_vec.data(),
    sizeof(AngularInfo) * h_ang_info_vec.size(),
    cudaMemcpyHostToDevice,
    stream);

  check_error << cudaStreamSynchronize(stream);
}

void DeviceVoxelMaps::calc_angular_info(const float max_norm, const Eigen::Vector3f& min_rpy, const Eigen::Vector3f& max_rpy, cudaStream_t stream) {
  std::vector<AngularInfo> ang_info_vec;
  ang_info_vec.resize(max_level_ + 1);

  for (int i = max_level_; i >= 0; i--) {
    const float cosine = 1 - (std::pow(h_info_vec[i].res, 2) / std::pow(max_norm, 2)) * 0.5;
    float ori_res = std::acos(std::max(cosine, static_cast<float>(-1.0)));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3f rpy_res_temp;
    rpy_res_temp.x() = ori_res <= (max_rpy.x() - min_rpy.x()) ? ori_res : 0.0;
    rpy_res_temp.y() = ori_res <= (max_rpy.y() - min_rpy.y()) ? ori_res : 0.0;
    rpy_res_temp.z() = ori_res <= (max_rpy.z() - min_rpy.z()) ? ori_res : 0.0;

    Eigen::Vector3f max_rpypiece;
    if (i == max_level_) {
      max_rpypiece = max_rpy - min_rpy;
    } else {
      max_rpypiece.x() = ang_info_vec[i + 1].rpy_res.x() != 0.0 ? ang_info_vec[i + 1].rpy_res.x() : max_rpy.x() - min_rpy.x();
      max_rpypiece.y() = ang_info_vec[i + 1].rpy_res.y() != 0.0 ? ang_info_vec[i + 1].rpy_res.y() : max_rpy.y() - min_rpy.y();
      max_rpypiece.z() = ang_info_vec[i + 1].rpy_res.z() != 0.0 ? ang_info_vec[i + 1].rpy_res.z() : max_rpy.z() - min_rpy.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() != 0.0 ? std::ceil(max_rpypiece.x() / rpy_res_temp.x()) : 1;
    num_division.y() = rpy_res_temp.y() != 0.0 ? std::ceil(max_rpypiece.y() / rpy_res_temp.y()) : 1;
    num_division.z() = rpy_res_temp.z() != 0.0 ? std::ceil(max_rpypiece.z() / rpy_res_temp.z()) : 1;
    ang_info_vec[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec[i].rpy_res.x() = num_division.x() != 1 ? max_rpypiece.x() / num_division.x() : 0.0;
    ang_info_vec[i].rpy_res.y() = num_division.y() != 1 ? max_rpypiece.y() / num_division.y() : 0.0;
    ang_info_vec[i].rpy_res.z() = num_division.z() != 1 ? max_rpypiece.z() / num_division.z() : 0.0;

    ang_info_vec[i].min_rpy.x() = ang_info_vec[i].rpy_res.x() != 0.0 && ang_info_vec[i + 1].rpy_res.x() == 0.0 ? min_rpy.x() : 0.0;
    ang_info_vec[i].min_rpy.y() = ang_info_vec[i].rpy_res.y() != 0.0 && ang_info_vec[i + 1].rpy_res.y() == 0.0 ? min_rpy.y() : 0.0;
    ang_info_vec[i].min_rpy.z() = ang_info_vec[i].rpy_res.z() != 0.0 && ang_info_vec[i + 1].rpy_res.z() == 0.0 ? min_rpy.z() : 0.0;
  }

  copy_ang_info_to_device(ang_info_vec, stream);
}

}  // namespace gpu