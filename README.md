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

If you have your own ros2 bag data, you can convert LiDAR msgs to pcd file with package below so that point cloud aligns in the direction of gravitational acceleration using IMU msgs.  
https://github.com/KOKIAOKI/ros2bag_to_pcd

## ROS2 test code
### 1. Build
- Viewer
```
# Install dependencies
sudo apt-get install -y libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev libboost-filesystem-dev libboost-program-options-dev

# Build and install Iridescence
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake ..
make -j
sudo make install
```

- gpu ros2 test
```
cd 3d_bbs/ros2_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. Download
Please download [ros2 test data]()

### 3. Config file setting

### 4. Run
First terminal
```
source install/
ros2 launch ros2_test
```

Second terminal
```
ros2 bag play <ros2 bag file path>
```


