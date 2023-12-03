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
### 1. Build
- GPU
```
cd 3d_bbs/test/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

### 2. Download
Please download [test data](https://drive.google.com/file/d/1JfdQjQ3-4qOmHtvYq8UafBCmbz45-F4Z/view?usp=drive_link).

### 3. Config file setting
Config file format is **3d-bbs/test/config/test.yaml**  
Please edit the config file as below:
1. Copy the **target** and **source** paths in the downloaded test_data to **target_clouds** and **source_clouds** items.
1. Create the output folder where you want to save the output pcd and copy the path to **output_folder** in test.yaml.
1. Test data work with default parameter values.  

![Alt text](figs/config_setting.gif)

### 4. Run
- GPU
```
cd 3d_bbs/test/build/
./gpu_test <config_file_path>
```

