import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from rostalker2interface.srv import *
from rostalker2interface.msg import *
from pathlib import Path
from rostalker2.worker_info_api import *
from rostalker2.worker_info_api import _get_node_info, _get_node_list, get_node_info

def transfer(self, to_name_or_id, from_name_or_id, item, arm_id): #TODO: switch from id to name_or_id for arm_id
	# Get destination node info
	to_entry = get_node_info(self, to_name_or_id)
	from_entry = get_node_info(self, from_name_or_id)

	# Error handling
	if(to_entry['type'] == '-1'): # Doesn't exist
		return self.status['ERROR']

	# Get name and id
	to_name = to_entry['name']
	to_id = to_entry['id']
	from_name = from_entry['name']
	from_id = from_entry['id']

	# Create request
	request = Transfer.Request()
	request.from_id = from_id
	request.from_name = from_name
	request.to_id = to_id
	request.to_name = to_name
	request.item = item
	
	# Wait for service TODO: make sure the arm exists with master first
	transfer_cli = self.create_client(Transfer, "/arm/%s/transfer"%arm_id)
	while(not transfer_cli.wait_for_service(timeout_sec=2.0)):
		self.get_logger().info("Service not available, trying again...")

	# Call client
	status = 10
	# TODO: add max attempts
	while(status == 10): #10 is WAITING
		future = transfer_cli.call_async(request)
		self.get_logger().info("Waiting on transfer")

		# Waiting for completion
		while(future.done() == False):
			time.sleep(1) # 1 second timeout
	
		if(future.done()):
			try:
				response = future.result()

				# Error handling TODO: currently only returns WAITING and SUCCESS
				if(response.status == response.ERROR or response.status == response.FATAL): # Some error occured
					raise Exception
			except Exception as e:
				self.get_logger().error("Error occured %r"%(e,))
				status = 10 # Retry
			else:
				if(response.status == response.SUCCESS):
					self.get_logger().info("Successfully transfered from name: %s to name: %s"%(from_name, to_name))
					status = self.status['SUCCESS']
				elif(response.status == response.WAITING):
					self.get_logger().info("Waiting on transfer from: %s to: %s"%(from_name, to_name))
					status = 10 # WAITING
	return self.status['SUCCESS']

# Transfers from the current robot to another robot
# TODO: DELETE this is old
def transfer_old(self, to_name_or_id, item, arm_id): #TODO: switch from id to name_or_id

	# Get destination node info
	to_entry = get_node_info(self, to_name_or_id)

	# Error handling
	if(to_entry['type'] == '-1'): # Doesn't exist
		return self.status['ERROR']

	# Get name and id
	to_name = to_entry['name']
	to_id = to_entry['id']

	# Create request
	request = Transfer.Request()
	request.from_id = self.id
	request.from_name = self.name
	request.to_id = to_id
	request.to_name = to_name
	request.item = item

	# Wait for service TODO: make sure the arm exists with master first
	transfer_cli = self.create_client(Transfer, "/arm/%s/transfer"%arm_id)
	while(not transfer_cli.wait_for_service(timeout_sec=2.0)):
		self.get_logger().info("Service not available, trying again...")

	# Call client
	future = transfer_cli.call_async(request)
	self.get_logger().info("Waiting on transfer")

	# Waiting for completion
	while(future.done() == False):
		time.sleep(1) # 1 second timeout
		self.get_logger().info("spinning") #TODO: DELETE
		
	if(future.done()):
		try:
			response = future.result()

			# Error handling
			if(response.status == response.ERROR or response.status == response.FATAL): # Some error occured
				raise Exception

		except Exception as e:
			self.get_logger().error("Error occured %r"%(e,))
			return self.status['ERROR']
		else:
			self.get_logger().info("Successfully transfered from name: %s to name: %s"%(from_name, to_name))
			return self.status['SUCCESS']


# Middleman function to segway to transfer call in retry function
def _transfer(args):
	return transfer(args[0], args[1], args[2], args[3], args[4]) #self, to_name_or_id, from_name_or_id, item, arm_id

# Called when waiting on a transfer to happen
# TODO: DELETE this is old
def wait_for_transfer_old(self, from_name_or_id, item, arm_id):

	# Get destination node info
	to_entry = get_node_info(self, from_name_or_id)

	# Error handling
	if(to_entry['type'] == '-1'): # Doesn't exist
		return self.status['ERROR']

	# Get name and id
	to_name = to_entry['name']
	to_id = to_entry['id']

	# Create request
	request = WaitForTransfer.Request()
	request.from_id = self.id
	request.from_name = self.name
	request.to_id = to_id
	request.to_name = to_name
	request.item = item

	# Wait for service TODO: make sure the arm exists with master first
	wait_for_transfer_cli = self.create_client(WaitForTransfer, "/arm/%s/wait_for_transfer"%arm_id)
	while(not wait_for_transfer_cli.wait_for_service(timeout_sec=2.0)):
		self.get_logger().info("Service not available, trying again...")

	# Call client
	future = wait_for_transfer_cli.call_async(request)
	self.get_logger().info("Waiting on transfer")

	# Waiting for completion
	while(future.done() == False):
		time.sleep(1) # 1 second timeout
		self.get_logger().info("spinning") #TODO: DELETE
	if(future.done()):
		try:
			response = future.result()

			# Error handling
			if(response.status == response.ERROR or response.status == response.FATAL): # Some error occured
				raise Exception
		except Exception as e:
			self.get_logger().error("Error occured %r"%(e,))
			return self.status['ERROR']
		else:
			self.get_logger().error("Successfully transfered from name: %s to name: %s"%(from_name, to_name))
			return self.status['SUCCESS']

# Middleman function to segway to wait for transfer call in retry function
# TODO: DELETE this is old
def _wait_for_transfer_old(args):
	return wait_for_transfer(args[0], args[1], args[2], args[3]) #self, from_name_or_id, item, arm_id

# dud main function
def main_null():
	print("This is not meant to have a main")
