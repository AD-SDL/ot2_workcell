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
		self.transfer_service = self.create_service(Transfer, "/arm/%s/transfer"%self.id, self.transfer_handler) # Handles transfer service requests

		# Initialization Complete
		self.get_logger().info("ID: %s name: %s initialization completed"%(self.id, self.name))

	# Handles transfer service requests
	def transfer_handler(self, request, response): #TODO
		# TODO if it sees another transfer request that also points to itself (to of the request)
		# Notify the user that a deadlock is occuring

		# only one transfer at a time
		self.arm_lock.acquire()

		# Get request information
		to_name = request.to_name
		to_id = request.to_id
		from_name = request.from_name
		from_id = request.from_id
		item = request.item

		# Create response
		response = Transfer.Resonse()

		# Get node (to/from) information
		to_entry = self.node_entry_request(to_name) # Search by name for now
		from_entry = self.node_entry_request(from_name)

		# error / warning
		if(to_entry['type'] == '-1'):
			response.status = response.ERROR # Error
		if(from_entry['type'] == '-1'):
			response.status = response.ERROR # Error

		# Set identifier and lock TODO


	# Create request to get node info
	def node_info_request(self, name_or_id):

		# Create request
		request = GetNodeInfo.Request()
		request.name_or_id = name_or_id

		# Client setup
		while(not self.get_node_info_cli.wait_for_service(timeout_sec=2.0)): # Wait for service to start
			self.get_logger().info("Service not available, trying again...")

		# Call service to get node info
		future = self.get_node_info_cli(request)
		self.get_logger().info("Waiting on node info for %s"%name_or_id)

		# Waiting on future
		while(future.done() == False):
			time.sleep(1) # 1 second timeout
		if(future.done()):
			entry = {'type':'-1'}
			try:
				response = future.result()
			except Exception as e:
				self.get_logger().error("Error occured %r"%(e,))
				return entry # Error
			else:
				self.get_logger().info("Node info for %s recieved"%name_or_id)
				entry['type'] = response.entry.type
				entry['name'] = response.entry.name
				entry['state'] = response.entry.state
				entry['id'] = response.entry.id
				return entry # All Good

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
