import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
import os

class OT2(Node):

	def __init__(self):
		super().__init__("ot2") #TODO: anonymous
		self.load_lock = Lock()
		self.run_lock = Lock()
		
		# Create clients
		self.register_cli = self.create_client(Register, 'register')
		while not self.register_cli.wait_for_service('register'):
			self.get_logger().info("Service not available, trying again...")
			time.sleep(2.0) # 2 seconds

		# Register with master
		register(self)

		# Create services
		self.load_service = self.create_service(LoadService, "/OT-2/%d/load"%self.id, self.load_handler) 
#		self.run_service = self.create_service(TODO, "/OT-2/%d/run"%self.id, self.run_handler)

	def load_handler(self, request, response):
		self.load_lock.acquire()
		name = request.name
		contents = request.contents
		my_file = Path(name)
		# Error checking
		if(my_file.is_file() and request.replace == False):
			self.get_logger().info("File %s already exists on the system and replacement is marked as false")
			self.load_lock.release()
			response.status = 1 # Error
			return response
		self.get_logger().info("Begginning load")
		try:
			f = open(name + ".py", "w")
			f.write(contents)
			f.close()
			os.chmod(name + ".py", 0x775)
		except Exception as e:
			self.logger().info("Error occured: %r"%(e,))
			reponse.status = 1 # Error
		else:
			self.logger().info("Loaded success")
			reponse.status = 0 # All good
		finally:
			self.load_lock.release()
			return response

	def run_handler(self, request, response):
		pass

def register(self):
	# Obtain our count
	req = Register.Request()
	req.type = "OT-2"
	self.future = self.register_cli.call_async(req)
	while rclpy.ok():
		rclpy.spin_once(self)
		if(self.future.done()):
			try:
				self.id = self.future.result().id
			except Exception as e:
				self.get_logger().info("Error occured: %r"%(e,))
			else:
				self.get_logger().info("Registration success")
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
