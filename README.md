# 3d_bbs
Fast and accurate 3D global localization using branch-and-bound scan matching.  
<img alt="overview" src="figs/overview.jpg" width="50%">

## Dependencies
- bbs3d (Lower versions are not tested)
  - Eigen3
  - CMake version 3.15 or higher
  - CUDA version 12.0 or higher (for GPU run)
- test
  - (All bbs3d dependencies)
  - PCL

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
- GPU (Please ignore the large number of warnings)
```
cd 3d_bbs/test/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

### 2. Download
Please download [test data](https://drive.google.com/file/d/1JfdQjQ3-4qOmHtvYq8UafBCmbz45-F4Z/view?usp=drive_link).

### 3. Config file setting
Please edit the config file as below:
- 3.1 Config file format is **3d-bbs/test/config/test.yaml**
- 3.2 Copy the **target** and **source** paths of the downloaded test data to **target_clouds** and **source_clouds** in test.yaml.
- 3.3 Create the output folder where you want to save the output pcd and copy the path to **output_folder** in test.yaml.
- 3.4 Test data work with default parameter values.

![Alt text](figs/config_setting.gif)

### 4. Run
- GPU
```
cd 3d_bbs/test/build/
./gpu_test [config_file_path]
```

