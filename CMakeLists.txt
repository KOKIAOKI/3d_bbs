cmake_minimum_required(VERSION 3.5)
project(bbs3d)

set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON) 

find_package(Eigen3 REQUIRED)

option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d
    SHARED bbs3d/src/gpu/bbs3d.cu
    bbs3d/src/gpu/calc_score.cu
    bbs3d/src/gpu/discreate_transformation.cu
    bbs3d/src/gpu/voxelmaps.cu
    bbs3d/src/gpu/stream_manager/stream_manager.cu
    bbs3d/src/gpu/stream_manager/check_error.cu
  )

  target_include_directories(gpu_bbs3d
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(gpu_bbs3d 
  )

  install(TARGETS gpu_bbs3d
    EXPORT gpu_bbs3d-export           
    LIBRARY DESTINATION lib
    INCLUDES  DESTINATION include
    PUBLIC_HEADER DESTINATION include/gpu_bbs3d)  
    

  install(EXPORT gpu_bbs3d-export
    FILE gpu_bbs3d-config.cmake
    DESTINATION share/cmake/gpu_bbs3d
    EXPORT_LINK_INTERFACE_LIBRARIES)

  install(DIRECTORY bbs3d/include/
    DESTINATION include/gpu_bbs3d)
endif()