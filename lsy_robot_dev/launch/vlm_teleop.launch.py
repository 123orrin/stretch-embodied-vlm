import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    respeaker_ros2_path = get_package_share_directory('respeaker_ros2')

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_core_path), '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'position', 'broadcast_odom_tf': 'False', 'fail_out_of_range_goal': 'True'}.items(),
    )

    d435i_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               stretch_core_path, 'launch'),
               '/d435i_high_resolution.launch.py'])
          )

    respeaker_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               respeaker_ros2_path, 'launch'),
               '/respeaker.launch.py'])
          )

    #vlm_teleop = Node(
    #    package='lsy_robot_dev',
    #    executable='vlm_teleop',
    #    output='screen',
    #    )

    #rviz_config_path = os.path.join(stretch_core_path, 'rviz', 'stretch_simple_test.rviz')

    #rviz_node = Node(
        #package='rviz2',
        #executable='rviz2',
        #arguments=['-d', rviz_config_path],
        #output='screen',
        #)

    return LaunchDescription([
        stretch_driver,
        d435i_launch,
        respeaker_launch,
        #vlm_teleop,
        #rviz_node,
        ])