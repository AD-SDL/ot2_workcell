import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from rostalker2.retry_functions import *
from rostalker2.register_functions import *
from rostalker2.register_functions import _register, _deregister_node

# TODO: arm is a shared resource has to be able to lock itself
# TODO: figure out how to integrate arm code

class Arm(Node):

	def __init__(self, name):
		# Node creation
		super().__init__("arm_" + name) # User specifies name

		# Lock creation
		self.arm_lock = Lock() # Only one can access arm at a time

		# Readabilty
		self.state = { #TODO maybe a sync with the master
			"BUSY":1,
			"READY":0
		}
		self.status = {
			"ERROR":1,
			"SUCCESS":0,
			"WARNING":2,
			"FATAL":3
		}

		# Path setup
		path = Path()
		self.home_location = str(path.home())
		self.module_location = self.home_location + "/ros2tests/src/OT2_Modules/"

		# Create clients
		self.register_cli = self.create_client(Register, 'register') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)
		self.deregister_cli = self.create_client(Destroy, 'destroy') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)

		# Register with master
		args = []
		args.append(self) # Self
		args.append("arm") # Type
		args.append(name) # Name
		status = retry(self, _register, 10, 1, args) # Setups up a retry system for a function, args is empty as we don't want to feed arguments 
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().fatal("Unable to register with master, exiting...")
			sys.exit(1) # Can't register node even after retrying

		# Create services
		self.transfer_service = self.create_service(Transfer, "/arm/%s/transfer"%self.id, self.transfer_handler) # Handles transfer service requests

		# Initialization Complete
		self.get_logger().info("ID: %s name: %s initialization completed"%(self.id, self.name))

	# Handles transfer service requests
	def transfer_handler(self, request, response): #TODO

		# Get request information
		to_name = request.to_name
		to_id = request.to_id
		from_name = request.from_name
		from_id = request.from_id
		item = request.item

		# Create response
		response = Transfer.Resonse()

		# Get node (to/from) information

		# error / warning



	# Create request to get node info
	def node_info_request(self, name_or_id):
		pass #TODO


def main(args=None):
	rclpy.init(args=args)

	if(len(sys.argv) != 2):
		print("need 1 arguments")
		sys.exit(1)
	name = str(sys.argv[1])

	arm_node = Arm(name)
	try:
		rclpy.spin(arm_node)
	except:
		arm_node.get_logger().error("Terminating...")

		# set up args
		args = []
		args.append(arm_node)
		status = retry(arm_node, _deregister_node, 10, 1.5, args) #TODO: handle status
		arm_node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
