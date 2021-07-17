import rclpy
from rclpy.node import Node
import sys
from workcell_interfaces.srv import *
from ot2_workcell_manager_client.retry_api import *

# registers a worker with the master node
def register(self, type, name):
	# Set node type
	self.type = type

	# Create request
	req = Register.Request()
	req.type = type # TODO: type check
	req.name = name

	# Create client and wait for service
	register_cli = self.create_client(Register, 'register')
	while not register_cli.wait_for_service(timeout_sec=2.0):
		self.get_logger().info("Service not available, trying again...")

	# Call
	future = register_cli.call_async(req)
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

	# Create client and wait for service
	deregister_cli = self.create_client(Destroy, 'destroy')
	while not deregister_cli.wait_for_service(timeout_sec=2.0):
		self.get_logger().info("Service not available, trying again...")

	# Call
	future = deregister_cli.call_async(req)
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

# Get id and name from master
def get_id_name(self):
	# Create a request
	request = GetId.Request()

	# Wait for service
	get_id_cli = self.create_client(GetId, '/%s/%s/get_id'%(self.type, self.name))
	while(not get_id_cli.wait_for_service(timeout_sec=2)):
		self.get_logger().info("Service not available, trying again...")

	# Call client
	future = get_id_cli.call_async(request)
	rclpy.spin_until_future_complete(self, future)
	if(future.done()):
		try:
			response = future.result()
			# name check
			if(not response.name == self.name):
				raise Exception()

			self.id = response.id
			self.type = response.type
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,))
			return self.status['ERROR']
		else:
			return self.status['SUCCESS']

def _get_id_name(args):
	return get_id_name(args[0]) # Self
