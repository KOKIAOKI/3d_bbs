# ROS2 Test code

## Dependencies
- (All bbs3d dependencies)
- ros2 humble

## Conditions for demonstrating 3D-BBS performance
- When the robot is stationary.
  - Reason: The error in the direction of gravitational acceleration increases while the robot is running.
- When the source point is completely included in the map point cloud.
  - Reason: Another pose that encompasses all source point cloud is estimated when source point cloud includes outside the map environment. 
  - Please use the downsampling and point cloud cutting tools.

## Test
You can choose either Rviz2 or iridescence viewer.  
Please download [ros2_test_data](https://drive.google.com/file/d/1AMnVRHrJ-cx_QdLE4AaGvBFMgEoLVWVR/view?usp=drive_link).

## A. Rviz2
### 1. Build
- Build ros2_test_rviz2 and click_loc
```
cd 3d_bbs/ros2_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros2_test_rviz2 click_loc
```

### 2. Config file setting
Config file format is 3d_bbs/ros2_test/config/ros2_test_rviz2.yaml

- **target_clouds**: Copy the target folder path containing the pcd files.
- **lidar_topic_name**: Support msg type: sensor_msgs::msg::PointCloud2
- **imu_topic_name**: Support msg type: sensor_msgs::msg::Imu  
ros2 test data work with default parameter values.

### 3. Run
Refer to this [video]().

**1. First terminal**
```
cd 3d_bbs/ros2_test
source install/setup.bash
ros2 launch ros2_test gpu_ros2_test_rviz2_launch.py
```

**2. Second terminal**
```
ros2 bag play <ros2 bag file path>
```

**3. Wait until this message is displayed.**
```
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
    [ROS2] 3D-BBS initialized
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
```
**4. Click the localize buttom**  
<img alt="overview" src="../figs/click_loc.png" width="10%">

## B. Iridescence
### 1. Build and install
- Build and Install Iridescence
Clone repository at Home directory.
```
# Install dependencies
sudo apt-get install -y libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev libboost-filesystem-dev libboost-program-options-dev

# Build and install Iridescence
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake ..
make -j8
sudo make install
```

- Build ros2_test_iridescence
```
cd 3d_bbs/ros2_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros2_test_iridescence
```

2. Congig file setting
Follow the same steps as **2. Congig file setting** in **A. Rviz2**.
 
 ### 3. Run
Refer to this [video]().

**1. First terminal**
```
cd 3d_bbs/ros2_test
source install/setup.bash
ros2 launch ros2_test_iridescence gpu_ros2_test_iridescence_launch.py
```

**2. Second terminal**
```
ros2 bag play <ros2 bag file path>
```

**3. Wait until this message is displayed.**
```
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
    [ROS2] 3D-BBS initialized
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
```
**4. Click the localize buttom**  
<img alt="overview" src="../figs/iridescence_click.png" width="10%">
