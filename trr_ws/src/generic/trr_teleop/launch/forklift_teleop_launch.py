from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    forklift_teleop_node = Node(
        package='trr_teleop',
        executable='forklift_teleop.py',
        name='forklift_teleop',
        output='screen',
        prefix='gnome-terminal --',  # Opens a new terminal for input/output
    )

    ld = LaunchDescription()
    ld.add_action(forklift_teleop_node)

    return ld
