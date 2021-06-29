import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from random import random
from rostalker2interface.srv import *

class Master(Node):
	
	def __init__(self):
		super().__init__("master_node")

		# Service setup 
		self.register_service = self.create_service(Register, 'register', self.handle_register)
		self.nodes = 1
		self.nodes_list = []
		self.register_lock = Lock()

		# Get Node
		while(len(self.nodes_list) == 0):
			self.get_logger().info("Waitting for nodes...")
			time.sleep(2) # 2 seconds
			rclpy.spin_once(self)

		# Initialization Complete
		self.get_logger().info("Master initialization complete")

	def load(self, name):

		# Select a node
		temp_node = self.nodes_list[int(random()*len(self.nodes_list))]
		type = temp_node['type']
		id = temp_node['id']
		# Client setup 
		self.load_cli = self.create_client(LoadService, "/%s/%s/load"%(type, id))
		while not self.load_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")
		try:
			f = open("test/"+name, "r")
			contents = f.read()
			f.close()
		except Exception as e:
			rospy.get_logger().info("Error occured: %r"%(e,))
		else:
			self.get_logger().info("File %s read"%name)
			load_request = LoadService.Request()
			load_request.name = name
			load_request.contents = contents
			load_request.replace = True
			self.future = self.load_cli.call_async(load_request)
			self.get_logger().info("Futured created")

			# Waiting on future
			rclpy.spin_until_future_complete(self, self.future)
			if(self.future.done()):
				try:
					response = self.future.result()
				except Exception as e:
					self.get_logger().info("Error occured %r"%(e,))
				if(response.status == 1):
					self.get_logger().error("Error occured in loading at %s for file %s"%(id, name))
				else:
					self.get_logger().info("Load succeeded")


	def handle_register(self, request, response):
		self.register_lock.acquire()
		response = Register.Response()
		dict = {
			"type":request.type,
			"id":"O"+str(self.nodes)
		}
		self.nodes_list.append(dict)
		response.status = 0
		response.id = dict['id']
		self.nodes += 1
		self.register_lock.release()
		return response

def main(args=None):
	rclpy.init(args=args)
	master = Master()
	master.load("test.py")
	master.destroy_node()
	rclpy.shutdown()

if __name__ ==  "__main__":
	main()
