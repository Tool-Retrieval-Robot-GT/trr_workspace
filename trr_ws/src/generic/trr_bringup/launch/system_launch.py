import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from xacro import process_file

def launch_perceptron_driver() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("perceptron_driver")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    perceptron_driver_pkg_dir = get_package_share_directory(
        "perceptron_driver"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                perceptron_driver_pkg_dir, "launch/perceptron_driver_launch.py"
            )
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

def launch_realsense_driver() -> IncludeLaunchDescription:
    pkg_realsense = get_package_share_directory('realsense2_camera')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_realsense, "launch", "rs_launch.py")),
        launch_arguments={
            "depth_module.depth_profile" : "1280x720x30",
            "pointcloud.enable" : "true"
        }.items()
    )

def launch_lidar_driver() -> Node:
    pkg_trr = get_package_share_directory('trr_bringup')
    param_file = os.path.join(pkg_trr, 'config', 'urg_params.yaml')
    return Node(
        package='urg_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[param_file]
    )

def launch_robot_state_publisher() -> Node:
    pkg_trr = get_package_share_directory('trr_bringup')
    robot_desc_path = os.path.join(pkg_trr, 'description', 'trr_robot.xacro')
    doc = process_file(robot_desc_path, mappings={'use_sim' : 'False'})
    robot_desc = doc.toprettyxml(indent='  ')

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_desc}]
    ) 

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(launch_perceptron_driver())
    ld.add_action(launch_realsense_driver())
    ld.add_action(launch_lidar_driver())
    ld.add_action(launch_robot_state_publisher())

    return ld
