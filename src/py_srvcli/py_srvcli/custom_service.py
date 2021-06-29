import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddThreeInts

class MinimalService(Node):

	def __init__(self):
		super().__init__('custom_service')
		self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)

	def add_three_ints_callback(self, request, response):
		response.sum = request.a + request.b + request.c
		self.get_logger().info("Incoming request recieved for %d + %d + %d"%(request.a, request.b, request.c))
		return response

def main(args=None):
	rclpy.init(args=args)
	minimal_service = MinimalService()
	rclpy.spin(minimal_service)
	minimal_service.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

