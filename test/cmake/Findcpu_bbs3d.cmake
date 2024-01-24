find_path(cpu_bbs3d_INCLUDE_DIRS cpu_bbs3d
  HINTS /usr/local/include)

find_library(cpu_bbs3d_LIBRARY NAMES cpu_bbs3d
  HINTS /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(cpu_bbs3d DEFAULT_MSG cpu_bbs3d_INCLUDE_DIRS cpu_bbs3d_LIBRARY)