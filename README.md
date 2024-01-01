# 3d_bbs
Fast and accurate 3D global localization using branch-and-bound scan matching.  
Please refer to our [paper](https://arxiv.org/abs/2310.10023).  

<img alt="overview" src="figs/overview.jpg" width="50%">


The latest implementation demonstrates faster processing times than those published in the paper.  
Specifically, when tested in a real environment with the following hardware configuration (Intel Core i7-10700K 3.8GHz, 32GB RAM, and NVIDIA GeForce RTX2060), the processing times are as follows: 
- Preparation
  - Paper: 9,272 ms on average
  - **Latest**: 3494 ms on average
  - **Using saved voxelmap**: 130 ms on average
- Localize
  - Paper: 878 ms on average
  - **Latest**: **189 ms** on average  

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
```shell script
git clone https://github.com/KOKIAOKI/3d_bbs.git
cd 3d_bbs
mkdir build && cd build
```

- CPU ver. & GPU ver. (Please ignore the large number of warnings)
```shell script
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
```

- CPU ver. only
```shell script
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
```

## Test code
See [test_code.md](./test/test_code.md)

## ROS2 test code
See [ros2_test_code.md](./ros2_test/ros2_test_code.md)
