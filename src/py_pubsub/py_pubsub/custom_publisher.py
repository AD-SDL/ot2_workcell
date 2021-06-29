import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Num

class MinimalPublisher(Node):
	
	def __init__(self):
		super().__init__("custom_publisher")
		self.publisher = self.create_publisher(Num, 'data', 10)
		timer_period = .1
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def timer_callback(self):
		msg = Num()
		msg.num = self.i
		self.publisher.publish(msg)
		self.get_logger().info("Published: %d"%msg.num)
		self.i += 1

def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)

	minimal_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

