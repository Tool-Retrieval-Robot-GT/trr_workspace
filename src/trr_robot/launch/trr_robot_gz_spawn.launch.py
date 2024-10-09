#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    trr_robot_path = get_package_share_directory("trr_robot")

    robot_desc = join(trr_robot_path, 'description', 'trr_robot.xacro')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'robot_description': robot_desc}],
        remappings=[
            ('/joint_states', 'trr_robot/joint_states'),
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "trr_robot",
            "-allow_renaming", "true",
            "-z", "0.28",
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/world/default/model/trr_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/trr_robot/joint_state', 'trr_robot/joint_states'),
            ('/odom', 'trr_robot/odom'),
            ('/scan', 'trr_robot/scan'),
            ('/depth_camera', 'trr_robot/depth_camera'),
            ('/cmd_vel', 'trr_robot/cmd_vel'),
            ('depth_camera/camera_info', 'trr_robot/depth_camera/camera_info'),
            ('/depth_camera/points', 'trr_robot/depth_camera/points'),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "depth_camera",
                    "--child-frame-id", "trr_robot/base_footprint/depth_camera"]
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_spawn_entity, transform_publisher, gz_ros2_bridge
    ])