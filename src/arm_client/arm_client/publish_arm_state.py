import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import time
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# Calls a service created by the arm manager to udpate the state of the arm
def update_arm_state(self, current_state):

	# Error checking
	if(current_state > 2 or current_state < 0):
		return self.status['ERROR'] # Error

	# Create a request
	msg = ArmStateUpdate()
	msg.state = current_state

	# Create client and wait for service
	arm_state_update_pub = self.create_publisher(ArmStateUpdate, "/arm/%s/arm_state_update"%self.id, 10)
	time.sleep(2) # wait for it to start

	# Call client
#	self.get_logger().info("Updating state")
	arm_state_update_pub.publish(msg)

	# No error checks without services :(
	return self.status['SUCCESS']

# Middleman function to segway from retry functions to update_arm_state
def _update_arm_state(args):
	return update_arm_state(args[0], args[1]) # self, current_state

def main_null():
	print("This is not meant to have a main function")
