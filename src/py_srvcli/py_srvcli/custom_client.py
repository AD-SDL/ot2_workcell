import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddThreeInts
import sys

class MinimalClientAsync(Node):
	
	def __init__(self):
		super().__init__("custom_client")
		self.cli = self.create_client(AddThreeInts, "add_three_ints")
		while not self.cli.wait_for_service("add_three_ints"):
			self.get_logger().info("Waiting for service...")
		self.req = AddThreeInts.Request()

	def send_request(self):
		self.req.a = int(sys.argv[1])
		self.req.b = int(sys.argv[2])
		self.req.c = int(sys.argv[3])
		self.future = self.cli.call_async(self.req)

def main(args=None):
	rclpy.init(args=args)
	minimal_client = MinimalClientAsync()
	minimal_client.send_request()

	if(rclpy.ok()):
		rclpy.spin_until_future_complete(minimal_client, minimal_client.future)
		if(minimal_client.future.done()):
			try:
				response = minimal_client.future.result()
			except Exception as e:
				minimal_client.get_logger().info("Error occured %r"%(e,))
			else:
				minimal_client.get_logger().info("Result is %d"%response.sum)

	minimal_client.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

