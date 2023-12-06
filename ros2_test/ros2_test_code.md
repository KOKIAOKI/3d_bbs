# ROS2 Test code

## Dependencies
- (All bbs3d dependencies)
- ros2 humble

## Test
You can choose either Rviz2 or iridescence viewer.
## A. Rviz2
### 1. Build
- Build ros2_test_rviz2 and click_loc
```
cd 3d_bbs/ros2_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros2_test_rviz2 click_loc
```

### 2. Download
Please download [ros2_test_data]().

### 3. Config file setting
Config file format is 3d_bbs/ros2_test/config/ros2_test_rviz2.yaml

-  **target_clouds**: Copy the target folder path containing the pcd files.
- Specify the ros2 bag topic:
  - **lidar_topic_name**: Support msg type: sensor_msgs::msg::PointCloud2
  - **imu_topic_name**: Support msg type: sensor_msgs::msg::Imu
- ros2 test data work with default parameter values.

### 4. Run
Refer to this [video]()

First terminal
```
cd 3d_bbs/ros2_test
source install/setup.bash
ros2 launch ros2_test gpu_ros2_test_rviz2_launch.py
```

Second terminal
```
ros2 bag play <ros2 bag file path>
```

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
make -j
sudo make install
```

- Build ros2_test_iridescence
```
cd 3d_bbs/ros2_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros2_test_iridescence
```

## 2. Download, 3. Congig file setting
Follow the same steps as **2. Download** and **3. Congig file setting** in **A. Rviz2**.
 
 ### 4. Run
Refer to this [video]()

First terminal
```
cd 3d_bbs/ros2_test
source install/setup.bash
ros2 launch ros2_test_iridescence gpu_ros2_test_iridescence_launch.py
```

Second terminal
```
ros2 bag play <ros2 bag file path>
```