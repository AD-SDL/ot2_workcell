import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from random import random
from rostalker2interface.srv import *
from pathlib import Path

# Only one master node can be running at anytime, or else you will cause issues 
class Master(Node):

	def __init__(self):
		# Node creation
		super().__init__("master_node") #TODO: Add in the ability to have multiple masters (maybe?)

		# Readability
		self.states = { #TODO: more states
			"BUSY":1,
			"READY":0
		}
		self.status = {
			"SUCCESS":0,
			"WARNING":2,
			"ERROR":1,
			"FATAL":3
		}

		# Path setup
		path = Path()
		self.home_location = str(path.home())
		self.module_location = self.home_location + "/ros2tests/src/OT2_Modules/"

		# Service setup 
		self.register_service = self.create_service(Register, 'register', self.handle_register) # registration service

		# Registration setup
		self.nodes = 0 # Total nodes registered
		self.nodes_list = [] # Information about all nodes registered: type, id, state

		# Lock setup
		self.register_lock = Lock()

		# Get Node
		# Master doesn't do anywork until there is a worker node to do stuff with
		while(len(self.nodes_list) == 0): 
			self.get_logger().info("Waitting for nodes...")
			rclpy.spin_once(self) # Allow node to register itself
			time.sleep(.5) # 2 seconds

		# Initialization Complete
		self.get_logger().info("Master initialization complete")

	# Loads filename to a worker node
	def load(self, name, replacement): #TODO: Add return statements, using self.status

		# Select a node
		target_node = self.nodes_list[int(random()*len(self.nodes_list))] #TODO switch from random assignment
		type = target_node['type'] # These will be needed to acess the service
		id = target_node['id']

		# Client setup 
		self.load_cli = self.create_client(LoadService, "/%s/%s/load"%(type, id)) # format of service is /{type}/{id}/{service name}
		while not self.load_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

		# Client ready
		try:
			f = open(self.module_location+name, "r") # TODO: create an actual well-known directory for modules to be placed in
			contents = f.read()
			f.close()
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR'] # Error
		else:
			self.get_logger().info("File %s read complete"%name)

			# Create a request
			load_request = LoadService.Request()
			load_request.name = name # File path: insert file name, the file path even though the same is given to the client to set up
			load_request.contents = contents # File string contents
			load_request.replace = replacement # If the file exists do we overwrite it?

			# Call service to load module
			self.future = self.load_cli.call_async(load_request)
			self.get_logger().info("Waiting for completion...")

			# Waiting on future
			rclpy.spin_until_future_complete(self, self.future)
			if(self.future.done()):
				try:
					response = self.future.result()
				except Exception as e:
					self.get_logger().error("Error occured %r"%(e,))
					return self.status['ERROR'] # Error
				else:
					# Error handling
					if(response.status == response.ERROR):
						self.get_logger().error("Error occured in loading at %s for file %s"%(id, name))
						return self.status['ERROR'] # Error
					elif(response.status == response.WARNING):
						self.get_logger().warning("Warning: File %s already exists on system %s"%(name, id))
						return self.status['WARNING'] # Warning
					else:
						self.get_logger().info("Load succeeded")
						return self.status['SUCCESS'] # All good

	# Registers a worker with the master so modules can be distrubuted
	def handle_register(self, request, response):
		# Lock
		self.register_lock.acquire()

		# Create response
		response = Register.Response()

		# Check type
		if(request.type == 'OT_2'):
			dict = {
				"type":request.type,
				"id":"O"+str(self.nodes),
				"state":self.states['READY'] #TODO: implement states
			}
			self.nodes_list.append(dict)
			self.get_logger().info("Registered ID: %s with master"%dict['id'])
		# TODO: more types
		else:
			self.get_logger().error("type %s not supported at this moment"%request.type)
			response.status = response.ERROR # Error
			self.register_lock.release() # Release lock before exiting
			return response

		# Create response
		response.status = response.SUCCESS
		response.id = dict['id'] # Send back the ID to the worker

		# Update Node information
		self.nodes += 1

		# Release lock and exit
		self.register_lock.release()
		return response

	#TODO
	def handle_destroy_worker(self, request, response):
		pass

# This is just for testing, this class can be used anywhere 
def main(args=None):
	rclpy.init(args=args)
	master = Master()
	status = master.load("test.py", False)
	master.get_logger().info("Status: %d"%status)
	master.destroy_node()
	rclpy.shutdown()

if __name__ ==  "__main__":
	main()
