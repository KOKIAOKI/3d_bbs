#include <gpu/stream_manager/stream_manager.hpp>
#include <cuda_runtime_api.h>

// StreamRoundRobin

StreamRoundRobin::StreamRoundRobin(int num_streams) {
  streams.resize(num_streams);
  for (int i = 0; i < num_streams; i++) {
    cudaStreamCreateWithFlags(&streams[i], cudaStreamNonBlocking);
  }
  cursor = 0;
}

StreamRoundRobin::~StreamRoundRobin() {
  for (int i = 0; i < streams.size(); i++) {
    cudaStreamSynchronize(streams[i]);
    cudaStreamDestroy(streams[i]);
  }
}

void StreamRoundRobin::sync_all() {
  for (int i = 0; i < streams.size(); i++) {
    cudaStreamSynchronize(streams[i]);
  }
}

cudaStream_t StreamRoundRobin::get_stream() {
  int stream = cursor++;
  return streams[stream % streams.size()];
}

// TempBufferManager

TempBufferManager::TempBufferManager(size_t init_buffer_size) {
  if (init_buffer_size) {
    buffers.push_back(std::make_shared<thrust::device_vector<char>>(init_buffer_size));
  }
}

TempBufferManager::~TempBufferManager() {}

char* TempBufferManager::get_buffer(size_t buffer_size) {
  if (buffers.empty() || buffers.back()->size() < buffer_size) {
    buffers.push_back(std::make_shared<thrust::device_vector<char>>(buffer_size * 1.2));
  }
  return thrust::raw_pointer_cast(buffers.back()->data());
}

void TempBufferManager::clear() {
  if (buffers.size() <= 1) {
    return;
  }

  buffers.erase(buffers.begin(), buffers.begin() + buffers.size() - 1);
}

void TempBufferManager::clear_all() {
  buffers.clear();
}

// StreamTempBufferRoundRobin

StreamTempBufferRoundRobin::StreamTempBufferRoundRobin(int num_streams, size_t init_buffer_size) {
  this->init_num_streams = num_streams;
  this->init_buffer_size = init_buffer_size;
  stream_roundrobin.reset(new StreamRoundRobin(num_streams));
}

StreamTempBufferRoundRobin::~StreamTempBufferRoundRobin() {}

void StreamTempBufferRoundRobin::sync_all() {
  stream_roundrobin->sync_all();
}

void StreamTempBufferRoundRobin::clear() {
  for (auto& buffer : buffer_map) {
    buffer.second->clear();
  }
}

void StreamTempBufferRoundRobin::clear_all() {
  for (auto& buffer : buffer_map) {
    buffer.second->clear_all();
  }
}

cudaStream_t StreamTempBufferRoundRobin::get_stream() {
  return stream_roundrobin->get_stream();
}

std::pair<CUstream_st*, TempBufferManager::Ptr> StreamTempBufferRoundRobin::get_stream_buffer() {
  cudaStream_t stream = stream_roundrobin->get_stream();
  auto found = buffer_map.find(stream);
  if (found == buffer_map.end()) {
    TempBufferManager::Ptr new_buffer(new TempBufferManager(init_buffer_size));
    found = buffer_map.insert(found, std::make_pair(stream, new_buffer));
  }
  return std::make_pair(stream, found->second);
}