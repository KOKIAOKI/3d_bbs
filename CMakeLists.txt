cmake_minimum_required(VERSION 3.5)
project(bbs3d)

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

add_library(cpu_bbs3d
  SHARED
  bbs3d/src/cpu_bbs3d/bbs3d.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
)

install(TARGETS cpu_bbs3d DESTINATION lib)

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d
    SHARED 
    bbs3d/src/gpu_bbs3d/bbs3d.cu
    bbs3d/src/gpu_bbs3d/calc_score.cu
    bbs3d/src/gpu_bbs3d/voxelmaps.cu
    bbs3d/src/gpu_bbs3d/stream_manager/check_error.cu
    bbs3d/src/gpu_bbs3d/voxelmaps_io.cu
  )

  install(TARGETS gpu_bbs3d DESTINATION lib)
endif()

install(DIRECTORY bbs3d/include/ DESTINATION include)