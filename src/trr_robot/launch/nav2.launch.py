import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_trr = get_package_share_directory('trr_robot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_trr, 'config', 'trr_map.yaml'),
            'params_file': os.path.join(pkg_trr, 'config', 'nav2_params.yaml'),
            'package_path': pkg_trr, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                pkg_trr,
                'rviz',
                'nav2.rviz'
            )
        ]
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_trr, 'config', 'amcl_params.yaml')],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_trr, 'config', 'trr_map.yaml')}],
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    remapper_node = Node(
        package='trr_robot',
        executable='remapper.py',
        name='remapper',
        output='screen',
    )  

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(remapper_node)

    return ld