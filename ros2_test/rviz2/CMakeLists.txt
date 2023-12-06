cmake_minimum_required(VERSION 3.5)
project(ros2_test_rviz2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(gpu_bbs3d REQUIRED)

if (BUILD_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

# CUDA
if(BUILD_CUDA)
  # viewer: Rviz2
  add_executable(gpu_ros2_test_rviz2
    src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp
    src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2_node.cpp
  )

  target_include_directories(gpu_ros2_test_rviz2 
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${gpu_bbs3d_INCLUDE_DIRS}
    ${Iridescence_INCLUDE_DIRS}
  )

  target_link_libraries(gpu_ros2_test_rviz2
    ${PCL_LIBRARIES}
    yaml-cpp
    ${gpu_bbs3d_LIBRARY}
    ${Iridescence_LIBRARIES}
  )

  ament_target_dependencies(gpu_ros2_test_rviz2
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    pcl_conversions
  )

  install(TARGETS gpu_ros2_test_rviz2 DESTINATION lib/${PROJECT_NAME})

endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()