import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
import os
import os.path
from os import path

class OT2(Node):

	def __init__(self):
		super().__init__("ot2") #TODO: anonymous
		self.load_lock = Lock()
		self.run_lock = Lock()
		
		# Create clients
		self.register_cli = self.create_client(Register, 'register')
		while not self.register_cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info("Service not available, trying again...")

		# Register with master
		self.register()

		# Create services
		self.load_service = self.create_service(LoadService, "/OT_2/%s/load"%self.id, self.load_handler) 
#		self.run_service = self.create_service(TODO, "/OT_2/%s/run"%self.id, self.run_handler)

	def load_handler(self, request, response):
		self.load_lock.acquire()
		name = request.name 
		contents = request.contents
		# Error checking
		if(path.exists(name) and request.replace == False):
			self.get_logger().info("File %s already exists on the system and replacement is marked as false")
			self.load_lock.release()
			response.status = 1 # Error
			return response
		self.get_logger().info("Beginning load")
		try:
			f = open(name, "w")
			f.write(contents)
			f.close()
			os.chmod(name, 0o777)
		except Exception as e:
			self.get_logger().info("Error occured: %r"%(e,))
			response.status = 1 # Error
		else:
			self.get_logger().info("Loaded success")
			response.status = 0 # All good
		finally:
			self.load_lock.release()
			return response

	def run_handler(self, request, response):
		pass

	def register(self):
		# Obtain our count
		req = Register.Request()
		req.type = "OT_2"
		self.future = self.register_cli.call_async(req)
		while rclpy.ok():
			rclpy.spin_once(self)
			if(self.future.done()):
				try:
					self.id = self.future.result().id
				except Exception as e:
					self.get_logger().info("Error occured: %r"%(e,))
				else:
					self.get_logger().info("Registration complete id: %s"%self.id)
				break
			self.get_logger().info("Waiting...")
			time.sleep(1) # 1 second

def main(args=None):
	rclpy.init(args=args)
	ot2node = OT2()
	rclpy.spin(ot2node)
	ot2node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
