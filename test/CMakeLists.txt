cmake_minimum_required(VERSION 3.5)
project(test)

# set C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lstdc++fs")

# set module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find bbs3d
find_package(bbs3d REQUIRED)

# find cuda (for gpu ver.)
option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --std=c++17 -Xcompiler -std=c++17)
endif()

# find openMP (for cpu ver.)
find_package(OpenMP)
if (OpenMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Common include directories
find_package(PCL REQUIRED)
find_package(small_gicp REQUIRED)
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
)

# voxelmaps saver
add_executable(saver
  src/voxelmaps_saver.cpp
)

target_link_libraries(saver
  ${COMMON_LIBRARIES}
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