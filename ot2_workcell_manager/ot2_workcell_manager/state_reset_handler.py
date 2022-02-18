# ROS Libaries 
import rclpy
from rclpy.node import Node

# Other Libraries
from threading import Thread
import time
from pathlib import Path

# Worker info API
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# ROS services and messages
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# This handles all state resets

# Current reset support
# Arm: done
# Master: TODO
# OT-2: done
class StateResetHandler(Node):

    def __init__(self):
        super().__init__("State_Reset_Handler")
        self.name = "state_reset_handler"
        self.type = "state_reset_handler"
        self.id = "S-1" # "S-1" like "M-1"

        # Readabilty
        self.state = {  # TODO maybe a sync with the master
            "BUSY": 1,
            "READY": 0,
            "ERROR": 2,
        }
        self.status = {
            "ERROR": 1,
            "SUCCESS": 0,
            "WARNING": 2,
            "FATAL": 3,
            "WAITING": 10,
        }

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/"

        # Initialization Complete
        self.get_logger().info(
            "State reset handler for ID: %s name: %s initialization completed"
            % (self.id, self.name)
        )

    # Publisher to reset the state of the transferhandler
    def node_state_reset(self, new_state, name_or_id):
        self.get_logger().info("Resetting arm state...")

        # Get Node info
        entry = get_node_info(self, name_or_id)

        # Error checking
        if(not (entry['type'] == 'arm' or entry['type'] == 'OT_2')):
            self.get_logger().error("Node doesn't exist or listed robot isn't a known type")
            return self.status['ERROR']
        # TODO: Provide warning if the state isn't in an error state

        # Get information from entry
        id = entry['id']
        type = entry['type']

        # Create msg and pub
        if(type == 'arm'):
            msg = ArmReset()
            reset_state_pub = self.create_publisher(ArmReset, "/arm/%s/arm_state_reset"%id, 10)
            time.sleep(1) # Sleep 1 second to wait for the publisher to finish
        elif(type == 'OT_2'):
            reset_state_pub = self.create_publisher(OT2Reset, "/OT_2/%s/ot2_state_reset"%id, 10)
            time.sleep(1) # Sleep 1 second to wait for the publisher to finish
            msg = OT2Reset()
        msg.state = new_state
        msg.id = id

        # Confirm
        confirmation = 'a'
        while(not (confirmation == 'Y' or confirmation == 'N')):
            confirmation = input("Confirmation required (Y/N): ")

        if(confirmation.strip() == 'N'):
            return self.status['WARNING']

        # Pub
        reset_state_pub.publish(msg)

        self.get_logger().info("State reset")

        return self.status['SUCCESS'] #TODO: Error handling

def work(self):
    temp = 'Y'
    while not temp == 'Q':
        temp = input("Y/N/Q for resetting node state: ")
        if(temp.strip() == 'Y'):
            node_name_or_id = input("Please enter node name or id: ")
            status = self.node_state_reset(self.state['READY'], node_name_or_id.strip()) # TODO: maybe do something with status

def main(args=None):
    rclpy.init(args=args)

    state_reset_handler = StateResetHandler()

    spin_thread = Thread(target = work, args = (state_reset_handler,))
    spin_thread.start()

    try:
        rclpy.spin(state_reset_handler)
    except:
        state_reset_handler.get_logger().fatal("Terminating...")

    spin_thread.join()
    state_reset_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
