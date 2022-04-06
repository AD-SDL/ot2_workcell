# ROS Libraries
import rclpy
from rclpy.node import Node

# Time library
import time

# ROS messages and services 
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

'''
    Internal interface for communication between the OT2 nodes to syncronize the state information
'''
# Function to transmit the heartbeat to the master
def heartbeat_transmitter(self):

    while rclpy.ok():    
        # Create a request for heartbeat message 
        msg = Heartbeat()
        msg.id = self.id

        # Create publisher object 
        transmit_heartbeat = self.creat_publisher(Heartbeat, "/heartbeat/heartbeat_update", 15)

        time.sleep(1)

        # Publish the heartbeat
        transmit_heartbeat.publish(msg)

    return 


def update_ot2_state(self, current_state, cur_block_name):

    # Error checking
    if not (current_state in self.state.values()):
        return self.status["ERROR"]  # Error

    # Create a request
    msg = OT2StateUpdate()
    msg.state = current_state
    msg.id = self.id
    msg.block_name = cur_block_name

    # Create client and wait for service
    ot2_state_update_pub = self.create_publisher(
        OT2StateUpdate, "/OT_2/ot2_state_update", 10
    )
    time.sleep(1)  # wait for it to start

    # Call client
    # 	self.get_logger().info("Updating state")
    ot2_state_update_pub.publish(msg)

    # No error checks without services :(
    return self.status["SUCCESS"]


# Middleman function to segway from retry functions to update_state
def _update_ot2_state(args):
    return update_ot2_state(args[0], args[1], args[2])  # self, current_state


def main_null():
    print("This is not meant to have a main function")
