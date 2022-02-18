# ROS libraries 
import rclpy
from rclpy.node import Node

# Time Library
import time

# ROS messages and services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

'''
    Calls the service to update the respective arm's state, this is currently set up so that only the armTransferHandler is able to call this function

    TODO: add in the ability for any function to be able to update the state of the arm or for more security only the master 
    TODO: doesn't confirm if the armManager is ready to recieve, or is down in some way shape or form. 
'''
def update_arm_state(self, current_state):

    # Error checking
    if not (current_state in self.state.values()):
        return self.status["ERROR"]  # Error

    # Create a request
    msg = ArmStateUpdate()
    msg.state = current_state
    msg.id = self.id

    # Create client and wait for service
    arm_state_update_pub = self.create_publisher(
        ArmStateUpdate, "/arm/%s/arm_state_update" % self.id, 10
    )
    time.sleep(1)  # wait for it to start

    # Call client
    arm_state_update_pub.publish(msg)

    # No error checks without services
    return self.status["SUCCESS"]

# Middleman function to segway from retry functions to update_arm_state
def _update_arm_state(args):
    return update_arm_state(args[0], args[1])  # self, current_state

def main_null():
    print("This is not meant to have a main function")
