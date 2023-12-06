import launch
import launch_ros.actions
import os

def generate_launch_description():
    config_file = os.path.join('rviz2/config/ros2_test.yaml')
    rviz_config_file = os.path.join('rviz2/rviz2/rviz2.rviz')

    gpu_ros2_test_rviz2 = launch_ros.actions.Node(
        package='ros2_test',
        executable='gpu_ros2_test_rviz2',
        output='screen',
        parameters=[{"config":config_file}],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return launch.LaunchDescription([gpu_ros2_test_rviz2, rviz_node])
