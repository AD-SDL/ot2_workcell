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

class ArmManager(Node):

	def __init__(self, name):
		# Node creation
		super().__init__("arm_manager_" + name) # User specifies name

		# Lock creation
#		self.arm_lock = Lock() # Only one can access arm at a time

		# Store who is doing the transfer and store respective locks
		self.cur_transfer = "" # identification of the 
		self.cur_wait = ""

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
		self.get_node_info_cli = self.create_client(GetNodeInfo, 'get_node_info') #TODO: maybe move this into respective functions
		self.get_node_list_cli = self.create_client(GetNodeList, 'get_node_list')

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
		self.get_id_service = self.create_service(GetId, "/arm/%s/get_id"%self.name, self.get_id_handler)

		# Initialization Complete
		self.get_logger().info("Arm Manager for ID: %s name: %s initialization completed"%(self.id, self.name))


	# Service to retrieve ID of the robot
	def get_id_handler(self, request, response):
		# Retrieve id and node information
		id = self.id
		name = self.name
		type = self.type

		# create response
		response = GetId.Response()
		response.id = id
		response.name = name
		response.type = type

		# Return response
		return reponse

	# TODO state of the machine

def main(args=None):
	rclpy.init(args=args)

	if(len(sys.argv) != 2):
		print("need 1 arguments")
		sys.exit(1)
	name = str(sys.argv[1])

	arm_manager_node = ArmManager(name)
	try:
		rclpy.spin(arm_manager_node)
	except:
		arm_node.get_logger().error("Terminating...")

		# set up args
		args = []
		args.append(arm_manager_node)
		status = retry(arm_manager_node, _deregister_node, 10, 1.5, args) #TODO: handle status
		arm_manager_node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
