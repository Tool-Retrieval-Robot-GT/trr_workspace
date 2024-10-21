import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_trr = get_package_share_directory('trr_robot')
    pkg_realsense = get_package_share_directory('realsense2_camera')

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'sensor_interface',
            default_value='serial',
            description='sensor_interface: supported: serial, ethernet')])

    def expand_param_file_name(context):
        param_file = os.path.join(pkg_trr, 'config', 'urg_params.yaml')
        if os.path.exists(param_file):
            return [SetLaunchConfiguration('param', param_file)]

    param_file_path = OpaqueFunction(function=expand_param_file_name)
    launch_description.add_action(param_file_path)

    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[LaunchConfiguration('param')]
    )
    launch_description.add_action(hokuyo_node)

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_realsense, "launch", "rs_launch.py")),
        launch_arguments={
            "depth_module.depth_profile" : "1280x720x30",
            "pointcloud.enable" : "true"
        }.items()
    )
    launch_description.add_action(realsense_node)
    
    return launch_description