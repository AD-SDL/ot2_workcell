import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
from rostalker2.retry_functions import *

# registers a worker with the master node
def register(self, type, name):
	# Set node type
	self.type = type

	# Create request
	req = Register.Request()
	req.type = type # TODO: type check
	req.name = name

	# Wait for service TODO client creation here
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
				self.get_logger().error("Registration of type %s failed, retrying..."%type)
				return self.status['ERROR']

			# Invalid id format check
			if(" " in response.id): #TODO: more invalid id checks
				self.get_logger().error("Invalid id format from master")
				#TODO: deregister the invalid node
				return self.status['ERROR']

			# All good
			self.id = future.result().id # Set ID
			self.name = name
		except Exception as e: # Error occured
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']
		else: # All good
			self.get_logger().info("Registration complete id: %s"%self.id)
			return self.status['SUCCESS']
	else: # This should never happen FATAL
		rospy.get_logger().fatal("future failed, critical failure")
		rospy.get_logger().fatal("Program is now terminating, PLEASE NOTE: System may be unstable")
		sys.exit(1)

# De-registers this node with the master node
def deregister_node(self):
	# Create Request
	req = Destroy.Request()
	req.type = self.type
	req.id = self.id

	# Wait for service TODO client creation here
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

# Middleman function to set up args
def _register(args):
	return register(args[0], args[1], args[2]) # self, type, name

# Middleman function to set up args
def _deregister_node(args):
	return deregister_node(args[0]) # self

def main_null():
	print("This is not meant to have a main function")
