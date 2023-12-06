# 3d_bbs
Fast and accurate 3D global localization using branch-and-bound scan matching.  
<img alt="overview" src="figs/overview.jpg" width="50%">

## Dependencies
- bbs3d (Lower versions are not tested)
  - Eigen3
  - CMake version 3.15 or higher
  - GPU version: CUDA version 12.0 or higher
- test
  - (All bbs3d dependencies)
  - PCL
- ros2 test
  - (All bbs3d dependencies)
  - ros2 humble

### Support Docker 🐳 

If nvidia driver is 525.60.11 or higher, try docker!  
For more information, you can check [docker_start.md](./docker/docker_start.md)  

## 3d_bbs core source code
### Build and Install
- GPU (Please ignore the large number of warnings)
```
git clone https://github.com/KOKIAOKI/3d_bbs.git
cd 3d_bbs
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
```

## Test code
See [test_code.md](./test/test_code.md)

## ROS2 test code
See [ros2_test_code.md](./ros2_test/ros2_test_code.md)
