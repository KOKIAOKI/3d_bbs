import launch
import launch_ros.actions
import os

def generate_launch_description():
    config_file = os.path.join('config/ros2_test.yaml')
    gpu_ros2_test_iridescence = launch_ros.actions.Node(
        package='ros2_test_iridescence',
        executable='gpu_ros2_test_iridescence',
        output='screen',
        parameters=[{"config":config_file}],
    )
    return launch.LaunchDescription([gpu_ros2_test_iridescence,])
