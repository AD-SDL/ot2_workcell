import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from rostalker2interface.srv import *
from pathlib import Path
from rostalker2.retry_functions import *
from rostalker2.worker_thread import worker_class

# Only one master node can be running at anytime, or else you will cause issues 
class Master(Node):

	def __init__(self):
		# Node creation
		super().__init__("master_node") #TODO: Add in the ability to have multiple masters (maybe?)

		# Lock setup
		self.node_lock = Lock() # This lock controls access to the self.nodes and self.nodes_list structure

		# Registration setup
		self.nodes = 0 # Total nodes registered
		self.nodes_list = [] # Information about all nodes registered: type, id, state
		self.node_wait_timeout = 2 # 2 seconds
		self.node_wait_attempts = 10 # 10 attempts before error thrown

		# Thread setup
		self.files_for_threads = [] # Maintains list of all the files each thread saved in threads_list needs to run
		self.threads_list = [] # Maintains all the thread objects

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
		self.destroy_service = self.create_service(Destroy, 'destroy', self.handle_destroy_worker) # Destroy worker service
		self.get_node_info_service = self.create_service(GetNodeInfo, 'get_node_info', self.handle_get_node_info) # Request is a name_or_id and returns all the information master has about that node
		self.get_node_list_service = self.create_service(GetNodeList, 'get_node_list', self.handle_get_node_list) # Blank request returns a list of all the nodes the master knows about

		# Client setup
		# TODO: see if any clients can be setup here 

		# Initialization Complete
		self.get_logger().info("Master initialization complete")

	# Loads filename to a worker node
	# parameters, name of file (path not needed done by worker), id of worker, and if the file already exists do we replace it or not? 
	def load(self, name, id, replacement):
		# Check node online?
		args = []
		args.append(id)
		status = retry(self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args) # retry function
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().error("Unable to find node %s"%id) # Node isn't registered
			return self.status['ERROR']
		else:
			self.get_logger().info("Node %s found"%id) # Found

		# Select a node
		try:
			# Get node information
			target_node = self.search_for_node(id) # See if id robot exists and the data

			# Error checking
			if(target_node['type'] == '-1'): # No such node
				self.get_logger().error("id: %s doesn't exist"%id)
				return self.status['ERROR']

#			target_node = self.nodes_list[int(random()*len(self.nodes_list))] # Random load assignment
			type = target_node['type'] # These will be needed to acess the service
			id = target_node['id']
		except Exception as e: 
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']

		# Client setup    (client can't be in the class as it constantly changes)
		#
		load_cli = self.create_client(LoadService, "/%s/%s/load"%(type, id)) # format of service is /{type}/{id}/{service name}
		while not load_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

		# Client ready
		try:
			f = open(self.module_location+name, "r")
			contents = f.read()
			f.close()
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR'] # Error

		self.get_logger().info("File %s read complete"%name) #Contents of file read

		# Create a request
		load_request = LoadService.Request()
		load_request.name = name # File path: insert file name, the file path even though the same is given to the client to set up
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

	

	# Registers a worker with the master so modules can be distrubuted
	def handle_register(self, request, response):

		# Create response
		response = Register.Response()

		# Check type
		if(request.type == 'OT_2'):
			dict = {
				"type":'OT_2',
				"id":"O"+str(self.nodes), # Can be searched along with name (each id must be unique)
				"state":self.states['READY'], #TODO: implement states
				"name":request.name
			}
			self.get_logger().info("Trying to register ID: %s name: %s with master"%(dict['id'], dict['name']))
		elif(request.type == 'arm'):
			dict = { #TODO: add more features that the master keeps about the node
				"type":'arm',
				"id":"A"+str(self.nodes), # Can be searched along with name (each id must be unique)
				"state":self.states['READY'], #TODO: implement states
				"name":request.name
			}
			self.get_logger().info("Trying to register ID: %s name: %s with master"%(dict['id'], dict['name']))
		# TODO: more types
		else:
			self.get_logger().error("type %s not supported at this moment"%request.type)
			response.status = response.ERROR # Error
			return response

		# Create response
		response.status = response.SUCCESS
		response.id = dict['id'] # Send back the ID to the worker

		# Lock: Critical section entry
		self.node_lock.acquire()

		# Update Node information: done last
		self.nodes += 1
		self.nodes_list.append(dict)

		# Release lock and exit
		self.node_lock.release()
		self.get_logger().info("Registration of ID: %s name: %s complete"%(dict['id'], dict['name']))
		return response

	# Removes node information upon service call
	def handle_destroy_worker(self, request, response): #TODO make it request name as well

		# Lock: Entering critical section
		self.node_lock.acquire()

		# Create response
		response = Destroy.Response()

		# Find id in nodes_list
		for i in range(0, self.nodes):
			dict = self.nodes_list[i]
			if(dict['id'] == request.id and dict['type'] == request.type):
				self.nodes_list.pop(i) # Remove from list
				self.get_logger().info("Removed id: %s of type: %s name: %s from nodes_list"%(dict['id'], dict['type'], dict['name']))
				response.status = response.SUCCESS
				self.node_lock.release()
				return response
			# Error checking
			elif(dict['id'] == request.id and not dict['type'] == request.type):
				self.nodes_list.pop(i) # Remove from list
				self.get_logger().info("Warning! id: %s name: %s doesn't match type in service request, type in request: %s, actual type: %s" % (dict['id'], dict['name'],  request.type, dict['type']))
				response.status = response.WARNING
				self.node_lock.release()
				return response

		# No id found in nodes_list
		self.get_logger().error("Unable to find id: %s of type: %s"%(request.id, request.type))
		response.status = response.ERROR
		self.node_lock.release()
		return response

	# Helper function to search nodes_list by id
	def search_for_node(self, id_or_name):
		# get lock entering critical section
		self.node_lock.acquire()

		for dict in self.nodes_list:
			if(dict['id'] == id_or_name or dict['name'] == id_or_name):
				# release lock and exit
				self.node_lock.release()
				return dict

		# leaving critical section
		self.node_lock.release()

		# Not found
		dict = { 'type':'-1', 'name':'-1', 'state':'-1', 'id':'-1' }
		return dict


	# Runs a module on the node (id)
	def run(self, file, id):
		# Check node online?
		args = []
		args.append(id)
		status = retry(self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args) # retry function
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().error("Unable to find node %s"%id) # Node isn't registered
			return self.status['ERROR']
		else:
			self.get_logger().info("Node %s found"%id) # Found

		# Get type
		node = self.search_for_node(id)
		type = node['type']

		# Client setup
		run_cli = self.create_client(Run, "/%s/%s/run"%(type, id)) # format of service is /{type}/{id}/{service name}
		while not run_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

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

	# This handles transfer service calls
	def transfer(self):
		pass #TODO

	# Function to segway to main function call
	def _transfer(self, args):
		pass #TODO

	# This handles transfer wait service calls
	def wait_for_transfer(self):
		pass #TODO

	# Function to segway to main function all
	def _wait_for_transfer(self, args):
		pass #TODO


	# Function to segway to main function call
	def _load(self, args):
		return self.load(args[0], args[1], args[2]) # File, id, replacement

	# Function to segway to main function call
	def _run(self, args):
		return self.run(args[0], args[1]) # File, id

	# this will both load and run a file at the robot id 
	#TODO: change to just load, transfer and run files onm OT-2s
	def load_and_run(self, file, id):
		# Load the module
		args = []
		args.append(file)
		args.append(id)
		args.append(True) # Auto update
		status = retry(self, self._load, 1, 0, args) # Calling retry function with 1 attempt, just want output information

		# Status check
		if(status == self.status['FATAL']):
			self.get_logger().fatal("Major error occured when attempting to run function")
			return self.status['FATAL'] # Fatal error
		elif(status == self.status['ERROR']):
			self.get_logger().error("Retry function stopped, either due to too many attempts or a bad status returned")
			return self.status['ERROR'] # Error
		elif(status == self.status['WARNING']):
			self.get_logger().warning("Warning thrown by retry function during execution, but ran to completion")
			# Continue
		else:
			self.get_logger().info("Function ran to completion")
			# Continue

		# Run the module
		args = []
		args.append(file)
		args.append(id)
		status = retry(self, self._run, 1, 0, args) # Calling retry function with 1 attempt to get output messages

		# Status check
		if(status == self.status['FATAL']):
			self.get_logger().fatal("Major error occured when attempting to run function")
			return self.status['FATAL'] # Fatal error
		elif(status == self.status['ERROR']):
			self.get_logger().error("Retry function stopped, either due to too many attempts or a bad status returned")
			return self.status['ERROR'] # Error
		elif(status == self.status['WARNING']):
			self.get_logger().warning("Warning thrown by retry function during execution, but ran to completion")
			return self.status['WARNING'] # Warnning thrown
		else:
			self.get_logger().info("Function ran to completion")
			return self.status['SUCCESS'] # All Good

	# Checks to see if the node/worker is ready (registered with the master)
	def node_ready(self, args):
		entry = self.search_for_node(args[0])
		if(entry['type'] == '-1'):
			self.get_logger().info("Waiting on node %s"%args[0])
			return self.status['ERROR']
		else:
			return self.status['SUCCESS']

	# Creates client that sends files to worker OT-2 to create threads
	def send_files(self, id, files): #self, id of robot, and files of current job

		# Check node online?
		args = []
		args.append(id)
		status = retry(self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args) # retry function
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().error("Unable to find node %s"%id) # Node isn't registered
			return self.status['ERROR']
		else:
			self.get_logger().info("Node %s found"%id) # Found

		# Select a node
		try:
			# Get node information
			target_node = self.search_for_node(id) # See if id robot exists

			# Error checking
			if(target_node['type'] == '-1'): # No such node
				self.get_logger().error("id: %s doesn't exist"%id)
				return self.status['ERROR']


			type = target_node['type'] # These will be needed to acess the service
			id = target_node['id']

		except Exception as e: 
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']

		# create client that calls file handler service on OT-2 module

		# Client setup
		send_cli = self.create_client(SendFiles, "/%s/%s/send_files"%(type, id)) # format of service is /{type}/{id}/{service name}
		while not send_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

		# Client ready
		#TODO: replacement parameter?
		# Create a request
		send_request = SendFiles.Request()
		#send_request.numFiles = len(files) # number of files to be sent to worker node
		send_request.files = files # string of file names list

		# Call Service to load module
		future = send_cli.call_async(send_request)
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

	# Reads from a setup file to run a number of files on a specified robot 
	def read_from_setup(self, file): #TODO: deadlock detection algorithm
		# Read from setup file and distrubute to worker threads
		# Read number of threads
		f = open(self.module_location + file, "r") # Open up file "setup" in well-known directory
		n = int(f.readline()) # First line should contain an integer, corresponds to number of threads

		# For each thread
		for i in range(n): # Starts reading names and files to be run
			# Get identification
			name_or_id = f.readline().strip() # Remove newline

			# Find entry for that id or name (spin to wait for it)
			args = []
			args.append(name_or_id)
			status = retry(self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args) # retry function
			if(status == self.status['ERROR'] or status == self.status['FATAL']):
				self.get_logger().error("Unable to find node %s"%name_or_id) # Node isn't registered
				return self.status['ERROR']
			else:
				entry = self.search_for_node(name_or_id) # get information about that node
				id = entry['id']
				self.get_logger().info("Node %s found"%name_or_id) # Found

			# Get files for the worker
			try:
				files = f.readline()

				self.files_for_threads.append(files) # should be the same index
			except Exception as e:
				self.get_logger().error("Reading from setup error: %r"%(e,))
				return self.status['ERROR'] # Error

			# Create thread
			# replace worker_class with send_files
			#temp_thread = worker_class(entry['name'], entry['id'], self, i) #name, id, master, index
			#self.threads_list.append(temp_thread) # Record information

			#files sent to worker OT-2 to become threads
			self.send_files(id, files)

			#TODO: set up lock? verify set up complete once files are sent

			# Setup complete for this thread
			self.get_logger().info("Setup complete for %s"%name_or_id)

		#TODO: migrate all below to ot2 class

		# New barrier for each thread (So we know when they all finish)
		# self.read_from_setup_barrier = threading.Barrier(n+1) # one for each thread and one for the main thread

		# Start running each thread
		# for item in self.threads_list:
			# item.start()

		# Waiting on finish
		# self.read_from_setup_barrier.wait()

		# Join each thread
		# for item in self.threads_list:
			# item.join()

		# Done
		self.get_logger().info("Setup file read and run complete")
		return self.status['SUCCESS']

	# Handles get node info service call
	def handle_get_node_info(self, request, response):
		#TODO: DELETE Debug
		self.get_logger().info("Node info request for %s"%request.name_or_id)

		# Get request
		entry = self.search_for_node(request.name_or_id)

		# Create a response
		response = GetNodeInfo.Response()
		response.entry.id = entry['id']
		response.entry.name = entry['name']
		response.entry.state = entry['state']
		response.entry.type = entry['type'] # Differ error checking back to caller

		# return response
		return response

	# Hanldes get the whole node list service call
	def handle_get_node_list(self, request, response):
		pass #TODO






def setup_thread_work(master):
	status = master.read_from_setup("setup") 

# TODO: Add a deregister master, so if the master disconnects or deregisters the workers can start waiting for a new master

# This is just for testing, this class can be used anywhere 
def main(args=None):
	rclpy.init(args=args)
	master = Master()

	# Create a thread to run setup_thread
	# spin_thread = Thread(target = setup_thread_work, args = (master,))
	# spin_thread.start()

	master.read_from_setup("setup")

#	status = master.load("module_test.py", "O0",  False)
#	status2 = master.run("module_test.py", "O0")
#	status = master.load_and_run("module_test.py", "O0")

	# Spin
	try:
		rclpy.spin(master)
	except Exception as e:
		master.get_logger().fatal("rclpy.spin failed, system in volatile state %r"%(e,))
	except:
		master.get_logger().fatal("Terminating...")

	# End
	# spin_thread.join()
	master.destroy_node()
	rclpy.shutdown()

if __name__ ==  "__main__":
	main()
