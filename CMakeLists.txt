cmake_minimum_required(VERSION 3.5)
project(bbs3d)

find_package(Eigen3 REQUIRED)

option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d
    SHARED bbs3d/src/gpu_bbs3d/bbs3d.cu
    bbs3d/src/gpu_bbs3d/calc_score.cu
    bbs3d/src/gpu_bbs3d/discreate_transformation.cu
    bbs3d/src/gpu_bbs3d/voxelmaps.cu
    bbs3d/src/gpu_bbs3d/stream_manager/check_error.cu
    bbs3d/src/gpu_bbs3d/voxelmaps_io.cu
  )

  target_include_directories(gpu_bbs3d
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
    $<INSTALL_INTERFACE:include>
  )

  install(TARGETS gpu_bbs3d DESTINATION lib)
endif()


install(DIRECTORY bbs3d/include/ DESTINATION include/bbs3d)