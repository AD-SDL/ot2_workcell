import rclpy 
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
	
	def __init__(self):
		super().__init__("minimal_service")
		self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

	def add_two_ints_callback(self, request, response):
		response.sum = request.a + request.b 
		self.get_logger().info("incoming request\na: %d b: %d" % (request.a, request.b))

		return response

def main(args=None):
	rclpy.init(args=args)
	
	minimal_service = MinimalService()
	rclpy.spin(minimal_service)
	minimal_service.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
