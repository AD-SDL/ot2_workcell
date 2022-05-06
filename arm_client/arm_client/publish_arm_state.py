# ROS libraries 
import rclpy
from rclpy.node import Node

# Time Library
import time

# ROS messages and services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
from datetime import datetime


'''
    Calls the service to update the respective arm's state, this is currently set up so that only the arm is able to call this function. 
'''
# Function to transmit the heartbeat to the master
def heartbeat_transmitter(self):
    """heartbeat_transmitter

        Description: Function to transmit the heartbeat to the master.
                     Creates and publishes the Heartbeat topic to the master.
                        
    """

    while rclpy.ok():    
        # Create a request for heartbeat message 
        msg = Heartbeat()
        msg.id = self.id

        # Create publisher object 
        transmit_heartbeat = self.create_publisher(Heartbeat, "/heartbeat/heartbeat_update", 10)

        # Dead check
        if(self.dead):
            return

        time.sleep(15)

        # Publish the heartbeat
        transmit_heartbeat.publish(msg)
        self.get_logger().info("--------- Heartbeat transmitted at %s ----------"% datetime.now())

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
        ArmStateUpdate, "/arm/arm_state_update", 10
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
