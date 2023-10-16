find_path(gpu_bbs3d_INCLUDE_DIRS gpu/bbs3d.cuh
HINTS /usr/local/include/gpu_bbs3d)

find_library(gpu_bbs3d_LIBRARY NAMES gpu_bbs3d
  HINTS /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gpu_bbs3d DEFAULT_MSG gpu_bbs3d_INCLUDE_DIRS gpu_bbs3d_LIBRARY)