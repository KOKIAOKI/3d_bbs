cmake_minimum_required(VERSION 3.5)
project(bbs3d)

find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

add_library(cpu_bbs3d
  SHARED
  bbs3d/src/cpu_bbs3d/bbs3d.cpp
  bbs3d/src/cpu_bbs3d/discreate_transformation.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
)

target_include_directories(cpu_bbs3d
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS cpu_bbs3d DESTINATION lib)

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

install(DIRECTORY bbs3d/include/ DESTINATION include)