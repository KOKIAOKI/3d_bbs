# Test code

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