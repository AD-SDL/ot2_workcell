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

class OT2(Node):

#TODO: run lock
	def __init__(self, name):

		# Node creation
		super().__init__("ot2_"+name) # Users specifies name

		# Lock creation
		self.file_lock = Lock() # Access to the file system

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

		self.work_list = [] # list of files list recieved from jobs
		self.work_index = 0

		# Node timeout info
		self.node_wait_timeout = 2 # 2 seconds
		self.node_wait_attempts = 10 # 10 attempts before error thrown

		# Create clients
		self.register_cli = self.create_client(Register, 'register') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)
		self.deregister_cli = self.create_client(Destroy, 'destroy') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)

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
		self.send_service = self.create_service(SendFiles, "/OT_2/%s/run"%self.id, self.recieve_files)
		#TODO: create service to unload and recieve items

		# Initialization Complete
		self.get_logger().info("ID: %s name: %s initialization completed"%(self.id, self.name))

	#Handles send_module service calls
	def recieve_files(self, request, response):

		# Get request information
		files = request.files

		# Create Response
		response = SendFiles.Response()

		# Warnings
		if(path.exists(file) and request.replace == False):
			self.get_logger().warning("File %s already exists on the system and replacement is false, upload terminating..."%name)
			response.status = response.WARNING # Warning: in this context file already exists on system
			return response
		if(request.replace == True):
			self.get_logger().warning("Replacement is set to True, file %s on this system will be replace with file from master"%name)
		
		# Begin reading file names
		self.get_logger().info("Reading file names")


		try:
			# Get lock
			self.file_lock.acquire()

			# Append files to work list
			#TODO: change, maybe run worker function?
			self.work_list.append(files)
			self.work_index = self.work_index + 1
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
			return response
		elif(not type == "OT_2"): # Wrong type
			self.get_logger().warning("The requested node type: %s doesn't match the node type of id: %s, but will still proceed"%(type, self.id))
		elif(path.exists(file) == False): # File doesn't exist
			self.get_logger().error("File: %s doesn't exist"%(file))
			response.status = response.ERROR
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


	# Overarching function. Parses through files in a job, loads and runs files
	def worker(self, index):
		#TODO: retrieve files from work list using method other than index
		files = self.work_list[self.work_index - 1]
		for file in files:
			# Debug information
			self.node_print("Running file %s"%file)

			# Load and run individual file
			status = self.load_and_run_ot2(self, file)

			# Error checking 

		self.node_print("thread %s done with work"%self.thread_ID)

		#TODO: Barrier?

	# load function, runs client that calls service on this node
	def load_ot2(self, file, replacement):
		#TODO: Check for node online and select node if all on same node?

		try:
			type = "OT_2"
			id = self.id
		except Exception as e: 
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']
		
		# Client setup
		load_cli = self.create_client(LoadService, "/%s/%s/load"%(type, id))
		while not load_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")
		
		# Client ready, read contents of file
		try:
			f = open(self.module_location+file, "r")
			contents = f.read()
			f.close()
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR'] # Error
		
		self.get_logger().info("File %s read complete"%name) #Contents of file read

		# Create Request
		load_request = LoadService.Request()
		load_request.name = file # File path: insert file name, the file path even though the same is given to the client to set up
		load_request.contents = contents # File string contents
		load_request.replace = replacement # If the file exists do we overwrite it?

		# Call service to load module
		future = load_cli.call_async(load_request)
		self.get_logger().info("Waiting for completion...")

		# Waiting on future
		while(future.done() == False):
			time.sleep(1) # timeout 1 second
		if(future.done()):
			try:
				response = future.result()
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



	# run function, runs client that calls service on this node
	def run_ot2(self, file):

		#TODO: Check for node online and select node if all on same node?

		try:
			type = "OT_2"
			id = self.id
		except Exception as e: 
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']
		
		# Client setup
		run_cli = self.create_client(Run, "/%s/%s/run"%(type, id)) # format of service is /{type}/{id}/{service name}
		while not run_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")
		
		# Create a request
		# Create a request
		req = Run.Request()
		req.type = type
		req.id = id
		req.file = file

		# Call service
		future = run_cli.call_async(req)
		self.get_logger().info("Waiting for completion...")

		# Waiting on future
		while(future.done() == False):
			time.sleep(1) # timeout 1 second
		if(future.done()):
			try:
				response = future.result()
			except Exception as e:
				self.get_logger().error("Error occured %r"%(e,))
				return self.status['ERROR'] # Error
			else:
				# Error checking
				if(response.status == response.ERROR):
					self.get_logger().error("Error occured when running file %s at id: %s"%(name, id))
					return self.status['ERROR'] # Error
				else:
					self.get_logger().info("Module run succeeded")
					return self.status['SUCCESS'] # All good



	#load_and_run funtion using retry 

def main(args=None):
	rclpy.init(args=args)

	if(len(sys.argv) != 2):
		print("need 1 arguments")
		sys.exit(1)
	name = str(sys.argv[1])

	ot2node = OT2(name)
	try:
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
