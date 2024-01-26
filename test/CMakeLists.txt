cmake_minimum_required(VERSION 3.5)
project(test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cpu_bbs3d REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (BUILD_CUDA)
  find_package(gpu_bbs3d REQUIRED)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# cpu_test
add_executable(cpu_test
  src/cpu_test.cpp
)

target_include_directories(cpu_test 
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(cpu_test
  ${PCL_LIBRARIES}
  ${cpu_bbs3d_LIBRARY}
  yaml-cpp
)

# CUDA
if(BUILD_CUDA)
  # gpu_test
  add_executable(gpu_test
    src/gpu_test.cpp
  )

  target_include_directories(gpu_test 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )

  target_link_libraries(gpu_test
    ${PCL_LIBRARIES}
    yaml-cpp
    ${gpu_bbs3d_LIBRARY}
  )

  # voxelmaps saver
  add_executable(saver
    src/voxelmaps_saver.cpp
  )

  target_include_directories(saver 
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
  )

  target_link_libraries(saver
    ${cpu_bbs3d_LIBRARY}
    ${Boost_LIBRARIES} 
  )
endif()