find_path(pointcloud_iof_INCLUDE_DIRS pointcloud_iof/load.hpp
HINTS /usr/local/include/pointcloud_iof)

find_library(pointcloud_iof_LIBRARY NAMES pointcloud_iof
  HINTS /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pointcloud_iof DEFAULT_MSG pointcloud_iof_INCLUDE_DIRS pointcloud_iof_LIBRARY)