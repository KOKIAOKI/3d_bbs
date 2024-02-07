cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cpu_bbs3d REQUIRED)
find_package(OpenMP)

if (OpenMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (BUILD_CUDA)
  find_package(gpu_bbs3d REQUIRED)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# Common include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# Common libraries
set(COMMON_LIBRARIES
  ${PCL_LIBRARIES}
  yaml-cpp
)

# cpu_test
add_executable(cpu_test
  src/cpu_test.cpp
)

target_link_libraries(cpu_test
  ${COMMON_LIBRARIES}
  ${cpu_bbs3d_LIBRARY}
)

# voxelmaps saver
add_executable(saver
  src/voxelmaps_saver.cpp
)

target_link_libraries(saver
  ${cpu_bbs3d_LIBRARY}
  ${Boost_LIBRARIES}
)

# CUDA
if(BUILD_CUDA)
  # gpu_test
  add_executable(gpu_test
      src/gpu_test.cpp
  )

  target_link_libraries(gpu_test
    ${COMMON_LIBRARIES}
    ${gpu_bbs3d_LIBRARY}
  )
endif()