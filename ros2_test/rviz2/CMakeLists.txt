cmake_minimum_required(VERSION 3.5)
project(ros2_test_rviz2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gpu_bbs3d REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# CUDA
if(BUILD_CUDA)
  ament_auto_add_executable(gpu_ros2_test_rviz2
    src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp
    src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2_node.cpp
  )

  target_include_directories(gpu_ros2_test_rviz2 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Iridescence_INCLUDE_DIRS}
  )

  target_link_libraries(gpu_ros2_test_rviz2
    ${PCL_LIBRARIES}
    yaml-cpp
    ${gpu_bbs3d_LIBRARY}
    ${Iridescence_LIBRARIES}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)