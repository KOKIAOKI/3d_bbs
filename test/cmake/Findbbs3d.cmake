find_path(bbs3d_INCLUDE_DIRS bbs3d HINTS /usr/local/include)
find_library(bbs3d_LIBRARY NAMES bbs3d HINTS /usr/local/lib)
find_library(cpu_bbs3d_LIBRARY NAMES cpu_bbs3d HINTS /usr/local/lib)
find_library(gpu_bbs3d_LIBRARY NAMES gpu_bbs3d HINTS /usr/local/lib)

set(bbs3d_LIBRARIES ${bbs3d_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(bbs3d DEFAULT_MSG bbs3d_INCLUDE_DIRS bbs3d_LIBRARIES)