#ifndef STREAMMANAGER_HPP
#define STREAMMANAGER_HPP

#include <iostream>
#include <vector>
#include <memory>
#include <atomic>

#include <thrust/device_vector.h>

#include <unordered_map>

typedef __device_builtin__ struct CUstream_st* cudaStream_t;

class StreamRoundRobin {
public:
  using cudaStream_t = CUstream_st*;

  StreamRoundRobin(int num_streams);
  ~StreamRoundRobin();

  void sync_all();

  cudaStream_t get_stream();

private:
  std::atomic_int cursor;

public:
  std::vector<cudaStream_t> streams;
};

class TempBufferManager {
public:
  using Ptr = std::shared_ptr<TempBufferManager>;

  TempBufferManager(size_t init_buffer_size = 0);
  ~TempBufferManager();

  char* get_buffer(size_t buffer_size);

  void clear();
  void clear_all();

private:
  std::vector<std::shared_ptr<thrust::device_vector<char, thrust::device_allocator<char>>>> buffers;
};

class StreamTempBufferRoundRobin {
public:
  StreamTempBufferRoundRobin(int num_streams = 16, size_t init_buffer_size = 1024);
  ~StreamTempBufferRoundRobin();

  cudaStream_t get_stream();
  std::pair<CUstream_st*, TempBufferManager::Ptr> get_stream_buffer();

  void sync_all();

  void clear();
  void clear_all();

private:
  size_t init_buffer_size;
  std::unordered_map<CUstream_st*, TempBufferManager::Ptr> buffer_map;

public:
  std::unique_ptr<StreamRoundRobin> stream_roundrobin;
  int init_num_streams;
};

#endif  // STREAMMANAGER_HPP
