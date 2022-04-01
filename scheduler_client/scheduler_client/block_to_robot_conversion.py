# ROS libraries 
import rclpy
from rclpy.node import Node

# ROS Messages and Services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *


'''
    This function converts a given block name to a robot name
'''
def convert_block_name(self, block_name):
    # checks
    if(block_name == "unknown"):
        self.get_logger().error("block_name of unknown not allowed, this should have been caught elsewhere")
        return self.status['ERROR']

    # Wait for service
    block_to_robot_cli = self.create_client(
        BlockToRobot, "/scheduler/block_to_robot"
    )
    while not block_to_robot_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Create request
    request = BlockToRobot.Request()
    request.block_name = block_name

    # Call service 
    future = block_to_robot_cli.call_async(request)

    # Waiting for completion
    while future.done() == False:
        time.sleep(1)  # 1 second timeout
    if future.done(): #TODO: Error handling
        response = future.result()
        return response.status, response.robot_name 
    
    # Error
    return response.ERROR, response.ERROR # should never happen   

def main_null():
    print("This function is not meant to have a main function")

if __name__ == "__main__":
    main_null()