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
from rostalker2interface.srv import *
from rostalker2interface.msg import *
from mastertalker_api.retry_api import *
from mastertalker_api.register_api import *
from mastertalker_api.register_api import _get_id_name
from mastertalker_api.worker_info_api import *
from mastertalker_api.worker_info_api import _get_node_info, _get_node_list, get_node_info
from random import random
from armtalker_api.publish_arm_state_api import *
from armtalker_api.publish_arm_state_api import _update_state

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
		self.arm_lock = Lock() # Only one can access arm at a time

		# Store who is doing the transfer and store respective locks
		self.transfer_queue = []
		self.completed_queue = []

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
			"FATAL":3
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
			sys.exit(1) #TODO: alert manager of error

		# Create services
		self.transfer_service = self.create_service(Transfer, "/arm/%s/transfer"%self.id, self.transfer_handler) # Handles transfer service requests

		# Initialization Complete
		self.get_logger().info("Arm Transfer handler for ID: %s name: %s initialization completed"%(self.id, self.name))

	# Handles transfer service requests
	def transfer_handler(self, request, response): #TODO: do something with item

		# Acquire lock
		self.arm_lock.acquire()

		# Get request
		to_name = request.to_name
		to_id = request.to_id
		from_name = request.from_name
		from_id = request.from_id
		item = request.item
		cur_node = request.cur_name
		other_node = request.other_name

		# Create response
		response = Transfer.Response()

		# Create identifier
		identifier_cur = from_name + " " + to_name + " " + item + " Node: " + cur_node
		identifier_other = from_name + " " + to_name + " " + item + " Node: " + other_node
#		self.get_logger().info("cur: %s   other: %s"%(identifier_cur, identifier_other)) #TODO: DELETE

		# Check to see if the transfer already completed
		completed = False
		for item in self.completed_queue:
			if(item == identifier_other): # Transfer already completed
				completed = True
				self.completed_queue.remove(item) # remove from completed queue
				break
		if(completed == True): # We are done
			response.status = response.SUCCESS
			self.arm_lock.release() # Realise lock
			return response

		# Check to see if other side is ready
		both_ready = False
		for item in self.transfer_queue:
			if(item == identifier_other): # Item is waiting can continue
				both_ready = True
				self.transfer_queue.remove(item) # delete from queue
				break

		# Check if in queue
		in_queue = False
		for item in self.transfer_queue:
			if(item == identifier_cur):
				in_queue = True
				break # in queue already

		# Adds current transfer identifier for the other side to verify
		if(in_queue == False):
			self.transfer_queue.append(identifier_cur)

		# If both aren't ready we return WAITING
		if(both_ready == False):
			response.status = response.WAITING # Still waiting on the other side
			self.arm_lock.release() # Realise lock
			return response

		# Update state and let it be know that we are busy
		args = []
		args.append(self)
		args.append(self.state['BUSY']) # busy
		status = retry(self, _update_state, 10, 2, args) # 10 tries 2 second timeout
		if(status == self.status['ERROR'] or status == self.status['FATAL']):
			self.get_logger().error("Unable to update state with manager, continuing but the state of the arm may be incorrect")

		# Both sides are ready complete the transfer and the transfer hasn't already been completed
		# Do the transfer
		self.current_state = self.state['BUSY']
		try:
			self.get_logger().info("Attempting to transfer complete transfer %s" % identifier_cur)
			time.sleep(2) #TODO actual transfer code
			self.get_logger().info("Transfer %s is complete"%identifier_cur)
		except Exception as e:
			# At this point it is safe to exit
			# The reason why is because we still only have one entry in the transfer_queue, and no entry
			# In the completed queue, which means the system is still in the same state it started in
			self.get_logger().error("Error occured: %r"%(e,))
			self.arm_lock.release() # Release lock
			response.status = response.ERROR # Error
			self.current_state = self.state['ERROR'] # ERROR state
			return response
		else:
			self.current_state = self.state['READY']
		finally: # No matter what after this the army is no longer busy 
			args = []
			args.append(self)
			args.append(self.current_state) # ready or error
			status = retry(self, _update_state, 10, 2, args) # 10 tries 2 second timeout
			if(status == self.status['ERROR'] or status == self.status['FATAL']):
				self.get_logger().error("Unable to update state with manager, continuing but the state of the arm may be incorrect")

		# if this point is reached the transfer is complete
		# Add to completed queue
		self.completed_queue.append(identifier_cur) # For the node waiting on it
		self.transfer_queue.remove(identifier_cur) # Remove our identifier from queue
		self.arm_lock.release() # Release lock
		response.status = response.SUCCESS
		return response

def main(args=None):
	rclpy.init(args=args)

#	if(len(sys.argv) != 2):
#		print("need 1 arguments")
#		sys.exit(1)
#	name = str(sys.argv[1])
	name = 'temp' #TODO: DELETE

	arm_transfer_node = ArmTransferHandler(name)
	try:
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
