import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
from rostalker2interface.msg import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from mastertalker_api.retry_api import *
from mastertalker_api.register_api import *
from mastertalker_api.register_api import _register, _deregister_node
from armtalker_api.transfer_api import *
from armtalker_api.transfer_api import _transfer

class OT2(Node):

	def __init__(self, name):

		# Node creation
		super().__init__("ot2_"+name) # Users specifies name

		# Lock creation
		self.file_lock = Lock() # Access to the file system
		self.run_lock = Lock() # Only one can access this OT-2 as a resource

		# readability
		self.state = { #TODO: sync with master
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
#		self.register_cli = self.create_client(Register, 'register') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)
#		self.deregister_cli = self.create_client(Destroy, 'destroy') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)

		# Register with master
		args = []
		args.append(self) # Self
		args.append("OT_2") # Type 
		args.append(name) # Name
		status = retry(self, _register, 10, 1, args) # Setups up a retry system for a function, args is empty as we don't want to feed arguments 
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().fatal("Unable to register with master, exiting...")
			sys.exit(1) # Can't register node even after retrying

		# Create services: Have to wait until after registration this way we will have the id
		self.load_service = self.create_service(LoadService, "/OT_2/%s/load"%self.id, self.load_handler) 
		self.run_service = self.create_service(Run, "/OT_2/%s/run"%self.id, self.run_handler)
		#TODO: create service to unload and recieve items

		# Initialization Complete
		self.get_logger().info("ID: %s name: %s initialization completed"%(self.id, self.name))

	# Handles load_module service calls
	def load_handler(self, request, response):

		# Get request information
		name = request.name
		file = self.module_location + name
		contents = request.contents

		# Create response
		response = LoadService.Response()

		# Warnings
		if(path.exists(file) and request.replace == False):
			self.get_logger().warning("File %s already exists on the system and replacement is false, upload terminating..."%name)
			response.status = response.WARNING # Warning: in this context file already exists on system
			return response
		if(request.replace == True):
			self.get_logger().warning("Replacement is set to True, file %s on this system will be replace with file from master"%name)

		# Begin loading module
		self.get_logger().info("Beginning load")
		try:
			# Get lock, Entering critical section
			self.file_lock.acquire()

			# Write to file and set permissions
			f = open(file, "w")
			f.write(contents)
			f.close()
			os.chmod(file, 0o777) # Exectuable permissions
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,))
			response.status = response.ERROR # Error
		else:
			self.get_logger().info("File %s loaded to OT2"%name)
			response.status = response.SUCCESS # All good
		finally:
			# Exiting critical section
			self.file_lock.release()
			return response

	# Handles run module service calls
	def run_handler(self, request, response):
		# Acquire lock
		self.run_lock.acquire()

		# Get request information
		type = request.type
		id = request.id
		file = self.module_location + request.file # Worker attaches to the well known location

		# Create response
		resposne = Run.Response()

		# Warnings / Errors
		if(not id == self.id): # Wrong ID
			self.get_logger().error("Request id: %s doesn't match node id: %s"%(id, self.id))
			response.status = response.ERROR
			self.run_lock.release() # Release lock
			return response
		elif(not type == "OT_2"): # Wrong type
			self.get_logger().warning("The requested node type: %s doesn't match the node type of id: %s, but will still proceed"%(type, self.id))
		elif(path.exists(file) == False): # File doesn't exist
			self.get_logger().error("File: %s doesn't exist"%(file))
			response.status = response.ERROR
			self.run_lock.release() # Release lock
			return response

		# Get lock, entering file critical section (Can't be reading when file is still being written)
		self.file_lock.acquire()

		# import module
		self.get_logger().info("Importing module...")
		try:
			# Loading and attaching the module to the program
			spec = importlib.util.spec_from_file_location(request.file, file)
			ot2Module = importlib.util.module_from_spec(spec)
			spec.loader.exec_module(ot2Module)
		except Exception as e:
			# Error
			self.get_logger().error("Error occured when trying to load module %s: %r"%(file,e,))
			response.status = response.ERROR # Error
			self.run_lock.release() # Release lock
			return response
		else:
			# All Good
			self.get_logger().info("Module %s loaded and attached to the program"%file)
		finally:
			# After exiting critical section release lock
			self.file_lock.release()

		# Running the module (The function ran is work()
		self.get_logger().info("Running module...")
		try:
			# Runs well-known function work()
			ot2Module.work()
		except Exception as e:
			# Error
			self.get_logger().error("Error occured when trying to run module %s: %r"%(file,e,))
			response.status = response.ERROR # Error
			return response
		else:
			# All good
			self.get_logger().info("Module %s successfully ran to completion"%file)
			response.status = response.SUCCESS
			return response
		finally:
			self.run_lock.release() # Release lock no matter what
#TODO: DELETE
def work(ot2node, name):
	args = []
	if(name == "bob"):
		args.append(ot2node)
		args.append("bob")
		args.append("alex")
		args.append("10")
		args.append("army")
		status = retry(ot2node, _transfer, 20, 4, args)
	if(name == "alex"):
		args.append(ot2node)
		args.append("bob")
		args.append("alex")
		args.append("10")
		args.append("army")
		status = retry(ot2node, _transfer, 20, 4, args)

def main(args=None):
	rclpy.init(args=args)

	if(len(sys.argv) != 2):
		print("need 1 arguments")
		sys.exit(1)
	name = str(sys.argv[1])

	ot2node = OT2(name)
	try:
		# TODO: DELETE
#		spin_thread = Thread(target = work, args = (ot2node,name,))
#		spin_thread.start()

		rclpy.spin(ot2node)
	except:
		ot2node.get_logger().error("Terminating...")

		# Setup args
		args = []
		args.append(ot2node) # Self
		status = retry(ot2node, _deregister_node, 10, 1.5, args) #TODO: handle status
		ot2node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
