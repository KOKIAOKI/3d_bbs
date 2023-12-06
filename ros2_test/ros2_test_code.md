# ROS2 Test code

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
source install/setup.bash
ros2 launch ros2_test gpu_ros2_test_launch.py
```

Second terminal
```
ros2 bag play <ros2 bag file path>
```

Third terminal
```
ros2 topic pub /click_loc std_msgs/msg/Bool "data: true" --once
```
