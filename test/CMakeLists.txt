cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpu_bbs3d REQUIRED)

if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# CUDA
if(BUILD_CUDA)
  add_executable(gpu_test
    src/gpu_bbs3d/gpu_test.cpp
  )

  target_include_directories(gpu_test 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${gpu_bbs3d_INCLUDE_DIRS}
  )

  target_link_libraries(gpu_test
    ${PCL_LIBRARIES}
    yaml-cpp
    ${gpu_bbs3d_LIBRARY}
  )
endif()