cmake_minimum_required(VERSION 3.5)
project(bbs3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lstdc++fs")

## find thirdparty/Eigen or Eigen3 package
option(USE_THIRDPARTY_EIGEN "Include thirdparty eigen" OFF)
file(GLOB entries "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen/*")
list(LENGTH entries num_entries)
message(STATUS "num_entries in thirdparty/Eigen: ${num_entries}")
if(num_entries EQUAL 0)
  find_package(Eigen3 REQUIRED)
else()
  set(USE_THIRDPARTY_EIGEN ON)
endif()

## find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## find CUDA
option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --std=c++17 -Xcompiler -std=c++17)
endif()

# Common include directories
if(USE_THIRDPARTY_EIGEN)
  include_directories(thirdparty/Eigen)
  message(STATUS "Use thirdparty Eigen")
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "Use EIGEN3_INCLUDE_DIR")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
  $<INSTALL_INTERFACE:include>
)

add_library(cpu_bbs3d SHARED bbs3d/src/bbs3d/cpu_bbs3d/bbs3d.cpp)

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d SHARED     
    bbs3d/src/bbs3d/gpu_bbs3d/bbs3d.cu
    bbs3d/src/bbs3d/gpu_bbs3d/voxelmaps.cu
    bbs3d/src/bbs3d/gpu_bbs3d/stream_manager/check_error.cu
  )
endif()

add_library(bbs3d SHARED $<TARGET_OBJECTS:cpu_bbs3d>)
if(BUILD_CUDA)
  target_sources(bbs3d PRIVATE $<TARGET_OBJECTS:gpu_bbs3d>)
endif()

install(TARGETS bbs3d cpu_bbs3d DESTINATION lib)
if(BUILD_CUDA)
  install(TARGETS gpu_bbs3d DESTINATION lib)
endif()

install(DIRECTORY bbs3d/include/ DESTINATION include)