from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    arm_manager = Node(
        package="arm_controller",
        executable="arm_manager",
        parameters=[{"name": "army"}],
        emulate_tty=True,
    )
    arm_transfer_handler = Node(
        package="arm_controller",
        executable="arm_transfer_handler",
        parameters=[{"name": "army"}],
        emulate_tty=True,
    )
    ld.add_action(arm_manager)
    ld.add_action(arm_transfer_handler)
    return ld
