import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = LaunchConfiguration("config", default ='/home/koki/3d_bbs/ros2_test/config/developer.yaml')
    gpu_ros2_test = launch_ros.actions.Node(
        package='ros2_test',
        executable='gpu_ros2_test',
        output='screen',
        parameters=[{"config":config}],
    )

    return launch.LaunchDescription([gpu_ros2_test])
