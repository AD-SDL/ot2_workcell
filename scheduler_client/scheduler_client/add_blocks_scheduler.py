# ROS libraries 
import rclpy
from rclpy.node import Node

# Time Library
import time

# ROS messages and services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# json library
import json

def add_blocks_scheduler(self, datastr): 
    data = json.loads(datastr)
    blocks = data['blocks']

    # Confirm type is correct 
    for block in blocks:
        if(isinstance(block['tasks'], str) == False):
            self.get_logger().error("the block %s, is not a string!"%(str(block),))
            return self.status['ERROR'] # block contains incorrect type 

    # Client setup
    add_block_cli = self.create_client(
        SchedulerWork, "/scheduler/%s/AddWork" % ("ana"),
    )  
    while not add_block_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # create request
    schedule_request = SchedulerWork.Request() 
    schedule_request.jsonblocks = datastr 

    # call async
    future = add_block_cli.call_async(schedule_request)

    # Waiting on future
    while future.done() == False:
        time.sleep(1)  # timeout 1 second
    if future.done():
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Error occured %r" % (e,))
            return self.status["ERROR"]  # Error
        else:
            # Error checking
            if response.status == response.ERROR:
                self.get_logger().error(
                    "Error adding blocks to scheduler"
                )
                return self.status["ERROR"]  # Error
            else:
                self.get_logger().info("Blocks added to scheduler")
                return self.status["SUCCESS"]  # All good

def main_null():
    print("This is not meant to have a main function")
