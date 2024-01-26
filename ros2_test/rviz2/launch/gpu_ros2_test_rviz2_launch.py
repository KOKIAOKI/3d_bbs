import launch
import launch_ros.actions
import os

def generate_launch_description():
    config_file = os.path.join('config/ros2_test.yaml')
    rviz_config_file = os.path.join('rviz2/rviz2_config/rviz2.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    gpu_ros2_test_rviz2 = launch_ros.actions.Node(
        package='ros2_test_rviz2',
        executable='gpu_ros2_test_rviz2',
        output='screen',
        parameters=[{"config":config_file}],
    )

    click_loc_node = launch_ros.actions.Node(
        package='click_loc',
        executable='click_loc',
        output='screen',
    )

    return launch.LaunchDescription([rviz_node, gpu_ros2_test_rviz2, click_loc_node])
