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

# Get master manager entry about the specific node
def get_node_info(self, name_or_id):

	# Create request
	request = GetNodeInfo.Request()
	request.name_or_id = name_or_id

	# Client setup
	while(not self.get_node_info_cli.wait_for_service(timeout_sec=2.0)): # Wait for service to start
		self.get_logger().info("Service not available, trying again...")

	# Call service to get node info
	future = self.get_node_info_cli(request)
	self.get_logger().info("Waiting on node info for %s"%name_or_id)

	# Waiting on future
	while(future.done() == False):
		time.sleep(1) # 1 second timeout
	if(future.done()):
		entry = {'type':'-1'}
		try:
			response = future.result()
		except Exception as e:
			self.get_logger().error("Error occured %r"%(e,))
			return entry # Error
		else:
			self.get_logger().info("Node info for %s recieved"%name_or_id)
			entry['type'] = response.entry.type
			entry['name'] = response.entry.name
			entry['state'] = response.entry.state
			entry['id'] = response.entry.id
			return entry # All Good

# Middleman to segway from retry functions or others to the get_node_info
def _get_node_info(args):
	return get_node_info(args[0], args[1]) #TODO: Current retry_functions do not support this type of output

#TODO
def get_node_list(self):
	pass

def _get_node_list(args):
	pass

def main_null():
	print("This is not meant to have a main function")
