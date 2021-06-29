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
		self.regiser_lock = Lock()

	def load(self, name):
		while(len(self.nodes_list) == 0):
			self.get_logger().info("Waitting for nodes...")
			time.sleep(2) # 2 seconds

		# Select a node
		temp_node = self.nodes_list[int(random()*len(self.nodes_list))]
		type = temp_node['type']
		count = temp_node['count']
		# Client setup 
		self.load_cli = self.create_client(LoadService, "/%s/%d/load"%(type, count))
		while not self.cli.wait_for_service("/%s/%d/load"%(type, count)):
			self.get_logger().info("Service not available, trying again...")
			time.sleep(2) # 2 seconds
		try:
			f = open(name+".py", "r")
			contents = f.read()
		except Exception as e:
			rospy.get_logger().info("Error occured: %r"%(e,))
		else:
			self.get_logger().info("File %s read"%name)
			load_request = load_service.Request()
			load_request.name = name
			load_request.contents = content
			load_request.replace = True
			self.future = self.load_cli.call_async(load_request)
			self.get_logger().info("Futured created")


	def handle_register(self, request, response):
		self.register_lock.acquire()
		response = register.Response()
		dict = {
			"type":request.type,
			"count":self.nodes
		}
		self.nodes_list.append(dict)
		response.status = 0
		response.id = self.nodes
		self.nodes += 1
		self.register_lock.release()
		return response

def main(args=None):
	rclpy.init(args=args)
	master = Master()
	master.load("test")
	rclpy.spin_until_future_complete(master.load, master.future)
	if(master.future.done()):
		try:
			response = master.future.result()
		except Exception as e:
			master.get_logger().info("Error occured %r"%(e,))

	master.destroy_node()
	rclpy.shutdown()

if __name__ ==  "__main__":
	main()
