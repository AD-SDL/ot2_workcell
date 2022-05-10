from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ot2_controller = Node(
        package="ot2_controller",
        executable="ot2_controller",
        output='screen',
        parameters=[{"name": "alex", "ip":"10.195.101.183", "port":"8085", "username":"alanwang"}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    protocol_handler = Node(
        package="ot2_controller",
        executable="protocol_manager",
        output='screen',
        parameters=[{"name": "alex", "ip":"10.195.101.183", "port":"8085", "username":"alanwang"}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    ld.add_action(ot2_controller)
    ld.add_action(protocol_handler)
    return ld
