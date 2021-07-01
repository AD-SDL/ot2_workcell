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

class OT2(Node):

	def __init__(self):

		# Node creation
		super().__init__("ot2") #TODO: anonymous

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

		# Create clients
		self.register_cli = self.create_client(Register, 'register') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)
		self.deregister_cli = self.create_client(Destroy, 'destroy') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)

		# Register with master
		status = self.retry(self.register, 10, 1) # Setups up a retry system for a function
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().fatal("Unable to register with master, exiting...")
			sys.exit(1) # Can't register node even after retrying

		# Create services: Have to wait until after registration this way we will have the id
		self.load_service = self.create_service(LoadService, "/OT_2/%s/load"%self.id, self.load_handler) 
		self.run_service = self.create_service(Run, "/OT_2/%s/run"%self.id, self.run_handler)

		# Initialization Complete
		self.get_logger().info("ID: %s initialization completed"%self.id)

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

	# TODO: Check the following code
	# import sys
	## the mock-0.3.1 dir contains testcase.py, testutils.py & mock.py
	#sys.path.append('/foo/bar/mock-0.3.1')
	#from testcase import TestCase
	#from testutils import RunTests
	#from mock import Mock, sentinel, patch

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


	# registers OT-2 with the master node
	def register(self):

		# Create request
		req = Register.Request()
		req.type = "OT_2"

		# Wait for service
		while not self.register_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")


		# Call
		future = self.register_cli.call_async(req)
		self.get_logger().info("Registering with master...")

		# Wait for completion
		rclpy.spin_until_future_complete(self, future)
		if(future.done()):
			try:
				# Get Response
				response = future.result()

				# Error checking
				if(response.status == response.ERROR or response.status == response.FATAL): #TODO: maybe add checks for valid status
					self.get_logger().error("Registration of type OT_2 failed, retrying...")
					return self.status['ERROR']

				# Invalid id format check
				if(" " in response.id): #TODO: more invalid id checks
					self.get_logger().error("Invalid id format from master")
					#TODO: deregister the invalid node
					return self.status['ERROR']

				# All good
				self.id = future.result().id # Set ID
			except Exception as e:
				self.get_logger().error("Error occured: %r"%(e,))
				return self.status['ERROR']
			else:
				self.get_logger().info("Registration complete id: %s"%self.id)
				return self.status['SUCCESS']
		else: # This should never happen
			rospy.get_logger().fatal("future failed, critical failure")
			rospy.get_logger().fatal("Program is now terminating, PLEASE NOTE: System may be unstable")
			sys.exit(1)

	# De-registers this node with the master node
	def deregister_node(self): #TODO swith to arg
		# Create Request
		req = Destroy.Request()
		req.type = "OT_2"
		req.id = self.id

		# Wait for service
		while not self.deregister_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

		# Call
		future = self.deregister_cli.call_async(req)
		self.get_logger().info("Deregistering with master...")

		# Wait for completion
		rclpy.spin_until_future_complete(self, future)
		if(future.done()):
			try:
				# Get Response
				response = future.result()

				# Error checking
				if(response.status == response.ERROR or response.status == response.FATAL):
					self.get_logger().error("Deregistration failed of id: %s, retrying..." % self.id)
					return self.status['ERROR']
				elif(response.status == response.WARNING):
					self.get_logger().warn("Deregistration success of id: %s, but warning thrown by master" % self.id)
					return self.status['WARNING']

				# All good
				self.get_logger().info("Deregistration complete")
				return self.status['SUCCESS']
			except Exception as e:
				self.get_logger().error("Error occured: %r"%(e,))
				return self.status['ERROR']
		else: # This should never happen
			rospy.get_logger().fatal("future failed, critical failure")
			rospy.get_logger().fatal("Program is now terminating, PLEASE NOTE: System may be unstable")
			sys.exit(1)

	# Tries until sucessfully executes, parameters: function, maximum number of attempts, and the timeout if failed (seconds)
	def retry(self, function, max_attempts, timeout): # TODO: TESTING
		attempts = 0
		status = 1 # Only works for functions that return a standard status signal
		while status != self.status['SUCCESS'] and status != self.status['WARNING'] and attempts < max_attempts: # Allowing for warnings to be passed
			try:
				# Attempting to run function
				status = function() # No debug information, function assumed to have it

				# Error checking
				if(status == self.status['ERROR'] or status == self.status['FATAL']):
					raise Exception
				if(status not in range(0, 4)):
					self.get_logger().error("Function doesn't return standard status output, stopping...")
					return self.status['ERROR'] # prematurely exit
			except:
				self.get_logger().error("Failed, retrying %s..."%str(function)) # Error occurred
			else:
				break

			# Increment counter and sleep
			attempts += 1
			time.sleep(timeout)

		# Error checking
		if(attempts == max_attempts): # If the loop is exited and we have a max attempt 
			return self.status['ERROR'] # Fatal error stopping attempts
		elif(status == self.status['SUCCESS'] or status == self.status['WARNING']):
			return self.status['SUCCESS'] # All good
		else: # Never should happen
			self.get_logger().fatal("Something unexpected occured in retry function")
			return self.status['FATAL'] # Let caller know a unknown error occured


def main(args=None):
	rclpy.init(args=args)
	ot2node = OT2()
	try:
		rclpy.spin(ot2node)
	except:
		ot2node.get_logger().error("Terminating...")
		ot2node.retry(ot2node.deregister_node, 10, 1.5) #TODO: handle status
		ot2node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
