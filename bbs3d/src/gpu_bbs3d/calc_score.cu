#include <gpu_bbs3d/bbs3d.cuh>

#include <cub/device/device_reduce.cuh>
#include <cub/iterator/transform_input_iterator.cuh>

#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {
struct lookup_voxel_map {
  const thrust::device_ptr<Eigen::Vector4i*> multi_buckets_ptrs;
  const thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  const thrust::device_ptr<const DiscreteTransformation> trans_ptr;
  const thrust::device_ptr<const Eigen::Vector3f> points_ptr;

  lookup_voxel_map(
    const thrust::device_ptr<Eigen::Vector4i*> multi_buckets_ptrs,
    const thrust::device_ptr<const VoxelMapInfo>& voxelmap_info_ptr,
    const thrust::device_ptr<const DiscreteTransformation>& trans_ptr,
    const thrust::device_ptr<const Eigen::Vector3f>& points_ptr)
  : multi_buckets_ptrs(multi_buckets_ptrs),
    voxelmap_info_ptr(voxelmap_info_ptr),
    trans_ptr(trans_ptr),
    points_ptr(points_ptr) {}

  __device__ int operator()(const int point_idx) const {
    // cast
    const DiscreteTransformation& trans = *thrust::raw_pointer_cast(trans_ptr);
    const VoxelMapInfo& voxelmap_info = *thrust::raw_pointer_cast(voxelmap_info_ptr + trans.level);
    const Eigen::Vector3f& point = thrust::raw_pointer_cast(points_ptr)[point_idx];
    const Eigen::Vector4i* buckets = thrust::raw_pointer_cast(multi_buckets_ptrs)[trans.level];

    // toransform
    Eigen::Vector3f translation(trans.x, trans.y, trans.z);
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f transed_point = rotation * point + translation;

    // coord to hash
    Eigen::Vector3i coord = (transed_point.array() * voxelmap_info.inv_res).floor().cast<int>();
    const std::uint32_t hash = (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);

    for (int i = 0; i < voxelmap_info.max_bucket_scan_count; i++) {
      const std::uint32_t bucket_index = (hash + i) % voxelmap_info.num_buckets;
      const Eigen::Vector4i bucket = buckets[bucket_index];

      if (bucket.x() != coord.x() || bucket.y() != coord.y() || bucket.z() != coord.z()){
        continue;
      }

      if (bucket.w() == 1) {
        return 1;
      }else{
        return 0;
      }
    }
    return 0;
  }
};

void BBS3D::create_cuda_graphs() {
  // Dummy transset
  std::vector<DiscreteTransformation> h_transset(graph_size_);
  for (int i = 0; i < graph_size_; i++) {
    h_transset[i] = DiscreteTransformation(0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  d_transset_stock_.clear();
  d_transset_stock_.shrink_to_fit();
  d_transset_stock_.resize(num_streams_);

  for (int i = 0; i < num_streams_; i++) {
    d_transset_stock_[i].resize(graph_size_);
    check_error << cudaMemcpyAsync(
      thrust::raw_pointer_cast(d_transset_stock_[i].data()),
      h_transset.data(),
      sizeof(DiscreteTransformation) * graph_size_,
      cudaMemcpyHostToDevice,
      stream);
  }

  // save storage
  void* d_storage = nullptr;
  size_t storage_bytes = 1024;
  check_error << cudaMallocAsync(&d_storage, storage_bytes, stream);

  // capture graph
  if (instances.size() != 0) {
    for (int i = 0; i < num_streams_; i++) {
      check_error << cudaGraphExecDestroy(instances[i]);
    }
    instances.clear();
    instances.shrink_to_fit();
  }
  instances.resize(num_streams_);

  for (int i = 0; i < num_streams_; i++) {
    check_error << cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);
    for (int j = 0; j < graph_size_; j++) {
      DiscreteTransformation* d_trans = thrust::raw_pointer_cast(d_transset_stock_[i].data() + j);

      lookup_voxel_map conversion_op(
        voxelmaps_ptr_->d_multi_buckets_ptrs_.data(),
        voxelmaps_ptr_->d_voxelmaps_info_.data(),
        d_transset_stock_[i].data() + j,
        d_src_points_.data());
      cub::TransformInputIterator<int, lookup_voxel_map, int*> itr(d_counts_, conversion_op);

      cub::DeviceReduce::Sum(d_storage, storage_bytes, itr, &d_trans->score, d_src_points_.size(), stream);
    }
    cudaGraph_t graph;
    check_error << cudaStreamEndCapture(stream, &graph);
    check_error << cudaGraphInstantiate(&instances[i], graph, NULL, NULL, 0);
    check_error << cudaGraphDestroy(graph);

    check_error << cudaStreamSynchronize(stream);
  }
  check_error << cudaFreeAsync(d_storage, stream);
  check_error << cudaStreamSynchronize(stream);
}

std::vector<DiscreteTransformation> BBS3D::calc_scores_by_graph(const std::vector<DiscreteTransformation>& h_transset) {
  std::vector<DiscreteTransformation> h_output(branch_copy_size_);
  for (int i = 0; i < num_streams_; i++) {
    auto inside_stream = stream_buffer_ptr_->get_stream();
    check_error << cudaMemcpyAsync(
      thrust::raw_pointer_cast(d_transset_stock_[i].data()),
      h_transset.data() + i * graph_size_,
      sizeof(DiscreteTransformation) * graph_size_,
      cudaMemcpyHostToDevice,
      inside_stream);
  }
  stream_buffer_ptr_->sync_all();

  // Calc Score
  for (int i = 0; i < num_streams_; i++) {
    auto inside_stream = stream_buffer_ptr_->get_stream();

    check_error << cudaGraphLaunch(instances[i], inside_stream);
  }
  stream_buffer_ptr_->sync_all();

  for (int i = 0; i < num_streams_; i++) {
    auto inside_stream = stream_buffer_ptr_->get_stream();

    check_error << cudaMemcpyAsync(
      h_output.data() + i * graph_size_,
      thrust::raw_pointer_cast(d_transset_stock_[i].data()),
      sizeof(DiscreteTransformation) * graph_size_,
      cudaMemcpyDeviceToHost,
      inside_stream);
  }
  stream_buffer_ptr_->sync_all();
  return h_output;
}

std::vector<DiscreteTransformation> BBS3D::calc_scores(const std::vector<DiscreteTransformation>& h_transset) {
  thrust::device_vector<DiscreteTransformation> d_transset(h_transset.size());
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_transset.data()),
    h_transset.data(),
    sizeof(DiscreteTransformation) * h_transset.size(),
    cudaMemcpyHostToDevice,
    stream);

  // Wait event
  cudaEvent_t outside_event;
  check_error << cudaEventCreate(&outside_event);
  check_error << cudaEventRecord(outside_event, stream);
  for (int i = 0; i < num_streams_; i++) {
    const auto& inside_stream = stream_buffer_ptr_->stream_roundrobin->streams[i];
    check_error << cudaStreamWaitEvent(inside_stream, outside_event, 0);
  }

  // Calc Score
  for (int i = 0; i < d_transset.size(); i++) {
    auto stream_buffer = stream_buffer_ptr_->get_stream_buffer();
    auto& inside_stream = stream_buffer.first;
    auto& buffer = stream_buffer.second;

    DiscreteTransformation* d_trans = thrust::raw_pointer_cast(d_transset.data() + i);

    lookup_voxel_map conversion_op(
      voxelmaps_ptr_->d_multi_buckets_ptrs_.data(),
      voxelmaps_ptr_->d_voxelmaps_info_.data(),
      d_transset.data() + i,
      d_src_points_.data());
    cub::TransformInputIterator<int, lookup_voxel_map, int*> itr(d_counts_, conversion_op);

    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;

    cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, itr, &d_trans->score, d_src_points_.size(), inside_stream);
    d_temp_storage = buffer->get_buffer(temp_storage_bytes);

    cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, itr, &d_trans->score, d_src_points_.size(), inside_stream);
  }

  // Wait event
  std::vector<CUevent_st*> events;
  events.resize(num_streams_, nullptr);
  for (int i = 0; i < num_streams_; i++) {
    const auto& inside_stream = stream_buffer_ptr_->stream_roundrobin->streams[i];
    check_error << cudaEventCreate(&events[i]);
    check_error << cudaEventRecord(events[i], inside_stream);
    check_error << cudaStreamWaitEvent(stream, events[i], 0);
  }

  // device to host
  std::vector<DiscreteTransformation> h_output(d_transset.size());
  check_error << cudaMemcpyAsync(
    h_output.data(),
    thrust::raw_pointer_cast(d_transset.data()),
    sizeof(DiscreteTransformation) * d_transset.size(),
    cudaMemcpyDeviceToHost,
    stream);
  check_error << cudaStreamSynchronize(stream);
  return h_output;
}
}  // namespace gpu