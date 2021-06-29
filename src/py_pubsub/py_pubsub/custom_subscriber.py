import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Num

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__("custom_subscriber")
		self.subscription = self.create_subscription(Num, 'data', self.listener_callback, 10)
		self.subscription

	def listener_callback(self, msg):
		self.get_logger().info("Heard %d"%msg.num)

def main(args=None):
	rclpy.init(args=args)
	minimal_subscriber = MinimalSubscriber()
	rclpy.spin(minimal_subscriber)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
