import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import time
import sys
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
from ot2_workcell_manager_client.retry_api import *
from ot2_workcell_manager_client.register_api import *
from ot2_workcell_manager_client.register_api import _get_id_name
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import _get_node_info, _get_node_list, get_node_info
from random import random
from arm_client.publish_arm_state import *
from arm_client.publish_arm_state import _update_arm_state

# TODO: figure out how to integrate arm code

class ArmTransferHandler(Node):
	def __init__(self, name):
		# Createa  temporary node so we can read in parameters
		super().__init__("Temp" + str(int(random()*17237967)))

		# Create parameters for name to be sent through
		self.declare_parameter('name', 'insert_arm_name_here') # 2nd arg is default value
		while(name == 'temp'):
			name = self.get_parameter('name').get_parameter_value().string_value
			time.sleep(1) # 1 second timeout

		# Node creation
		super().__init__("arm_transfer_handler_" + name) # User specifies name
		self.name = name
		self.type = 'arm'

		# Lock creation

		# Readabilty
		self.state = { #TODO maybe a sync with the master
			"BUSY":1,
			"READY":0,
			"ERROR":2,
		}
		self.status = {
			"ERROR":1,
			"SUCCESS":0,
			"WARNING":2,
			"FATAL":3,
			"WAITING":10,
		}

		# State of the arm
		self.current_state = self.state['READY']

		# Path setup
		path = Path()
		self.home_location = str(path.home())
		self.module_location = self.home_location + "/ros2tests/src/OT2_Modules/"

		# Create clients

		# Get ID and confirm name from manager
		args = []
		args.append(self)
		status = retry(self, _get_id_name, 5, 2, args) # 5 retries 2 second timeout
		if(status == self.status['ERROR']):
			self.get_logger().fatal("Unable to get id from arm manager, exiting...")
			# Alert manager of error
			self.current_state  = self.state['ERROR']
			self.set_state()

		# Create services

		# Initialization Complete
		self.get_logger().info("Arm Transfer handler for ID: %s name: %s initialization completed"%(self.id, self.name))

	# retrieves the next transfer to run in the queue
	# No need to lock the arm this is done by the manager
	def get_next_transfer(self):
		# Get the next transfer
		# Create client and wait for service
		get_next_transfer_cli = self.create_client(GetNextTransfer, "/arm/%s/get_next_transfer"%self.id)
		while(not get_next_transfer_cli.wait_for_service(timeout_sec=2.0)):
			self.get_logger().info("Service not available, trying again...")

		# Create request
		request = GetNextTransfer.Request()

		# Call the cli
		next_transfer = ""
		future = get_next_transfer_cli.call_async(request)
		while(future.done() == False):
			time.sleep(1) # 1 second timeout
		if(future.done()):
			try:
				response = future.result()

				if(response.status == response.ERROR or response.status == response.FATAL):
					raise Exception
				elif(response.status == response.WAITING):
					return self.status['WAITING']
			except Exception as e:
				self.get_logger().error("Error occured %r"%(e,))
				return self.status['ERROR']
			else:
				next_transfer = response.next_transfer.splitlines() # Breaks up based on newlines

		# Get transfer data
		transfer_request = next_transfer[0].split() # Split based on whitespaces
		from_name = transfer_request[0]
		to_name = transfer_request[1]

		# Get node information
		to_entry = get_node_info(self, to_name)
		from_entry = get_node_info(self, from_name)

		# Error checking
		if(to_entry['type'] == '-1'): # Doesn't exist
			self.get_logger().error("node: %s doesn't exist with master"%to_name)
			return self.status['ERROR']
		if(from_entry['type'] == '-1'): # Doesn't exist
			self.get_logger().error("node: %s doesn't exist with master"%from_name)
			return self.status['ERROR']

		# Update state and let it be know that we are busy
		self.current_state = self.state['BUSY']
		self.set_state()

		# Do the transfer
		try:
			self.get_logger().info("Attempting to transfer complete transfer %s" % str(next_transfer))
			time.sleep(2) #TODO actual transfer code
			self.get_logger().info("Transfer %s is complete"%str(next_transfer))
		except Exception as e:
			# At this point it is safe to exit
			# The reason why is because we still only have one entry in the transfer_queue, and no entry
			# In the completed queue, which means the system is still in the same state it started in
			self.get_logger().error("Error occured: %r"%(e,))
			self.current_state = self.state['ERROR'] # ERROR state
			return self.status['ERROR']
		else:
			self.current_state = self.state['READY']
		finally: # No matter what after this the army is no longer busy
			self.set_state()

		# Add to completed queue

		# Create pub
		completed_transfer_pub = self.create_publisher(CompletedTransfer,"/arm/%s/completed_transfer"%self.id, 10)
		time.sleep(1) # Wait for it

		# Create msg
		msg = CompletedTransfer()
		msg.identifier_cur = next_transfer[0]
		msg.identifier_other = next_transfer[1]

		# Publish
		completed_transfer_pub.publish(msg)

		return self.status['ERROR']

	# Helper function
	def set_state(self):
		args = []
		args.append(self)
		args.append(self.current_state)
		status = retry(self, _update_arm_state, 10, 2, args)
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().error("Unable to update state with manager, continuing but the state of the arm may be incorrect")

	# Function to constantly poll manager queue for transfers
	def run(self):
		# Runs every 3 seconds
		while(rclpy.ok()):
			status = self.get_next_transfer()
			time.sleep(3)

def main(args=None):
	rclpy.init(args=args)

#	if(len(sys.argv) != 2):
#		print("need 1 arguments")
#		sys.exit(1)
#	name = str(sys.argv[1])
	name = 'temp' #TODO: DELETE

	arm_transfer_node = ArmTransferHandler(name)
	try:
		spin_thread = Thread(target = arm_transfer_node.run, args = ())
		spin_thread.start()

		rclpy.spin(arm_transfer_node)
	except Exception as e:
		arm_transfer_node.get_logger().fatal("Error %r"%(e,))
	except:
		arm_transfer_node.get_logger().error("Terminating...")

	# End
	arm_transfer_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
