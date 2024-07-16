cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lstdc++fs")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(bbs3d REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP)

if (OpenMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --std=c++17 -Xcompiler -std=c++17)
endif()

# Common include directories
find_package(PCL REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${bbs3d_INCLUDE_DIRS}
)

# Common libraries
set(COMMON_LIBRARIES
  ${PCL_LIBRARIES}
  ${bbs3d_LIBRARIES}
  small_gicp::small_gicp
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
  ${COMMON_LIBRARIES}
  ${cpu_bbs3d_LIBRARY})

# CUDA
if(BUILD_CUDA)
  # gpu_test
  add_executable(gpu_test
      src/gpu_test.cpp
  )

  target_link_libraries(gpu_test
    ${COMMON_LIBRARIES}
    ${cpu_bbs3d_LIBRARY}
    ${gpu_bbs3d_LIBRARY}
  )
endif()