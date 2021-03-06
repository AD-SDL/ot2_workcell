# ROS libraries 
import rclpy
from rclpy.node import Node

# Time Library 
import time

# ROS Messages and Services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# OT2_workcell_manager API
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# Scheduler client
from scheduler_client.block_to_robot_conversion import *
from scheduler_client.block_to_robot_conversion import convert_block_name
'''
 This is an API that will begin a transfer request from_node to_node on the designated arm_name_or_id. 

 Note: item is currently unused as we don't have anything programmed for it yet so it is just ignored
'''
def load_transfer(
    self, block_name1, block_name2, item, arm_name_or_id
):  #TODO: do something with item
    # Robot names
    from_name_or_id = ""
    to_name_or_id = ""

    # If mappings exist
    block1_status, block1_robot = convert_block_name(self, block_name1)
    block2_status, block2_robot = convert_block_name(self, block_name2)

    # Block1 mapping exists
    if(block1_status == self.status['SUCCESS']):
        from_name_or_id = block1_robot
    else: # doesn't exist
        return self.status['WAITING'] # alert WAITING

    # Block2 mapping exists
    if(block2_status == self.status['SUCCESS']):
        to_name_or_id = block2_robot
    else: # doesn't exist
        return self.status['WAITING']

    # Get destination node info
    to_entry = get_node_info(self, to_name_or_id)
    from_entry = get_node_info(self, from_name_or_id)

    # Get arm info
    arm_entry = get_node_info(self, arm_name_or_id)

    # Error handling
    if to_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % to_name_or_id)
        return self.status["ERROR"]
    if from_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % from_name_or_id)
        return self.status["ERROR"]
    if arm_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % arm_name_or_id)
        return self.status["ERROR"]

    # Get name and id
    to_name = to_entry["name"]
    to_id = to_entry["id"]
    from_name = from_entry["name"]
    from_id = from_entry["id"]
    arm_name = arm_entry["name"]
    arm_id = arm_entry["id"]

    # Create request
    request = LoadTransfer.Request()
    request.from_id = from_id
    request.from_name = from_name
    request.to_id = to_id
    request.to_name = to_name
    request.item = item

    '''
        Setup request information, if you are master you can override and directly initiate a transfer or else it has to come from one of the nodes involved. 

        The master bypass is a bit iffy and probably will be removed as the master will need to make 2 separate transfer requests based on the way it is currently setup. 
    '''
    if self.type == "master": 
        # Bypass restrictions on who does what transfer
        request.cur_name = "master"  # Only the master can remove it from the queue
        request.other_name = "master"
    else:
        request.cur_name = self.name
        if self.name == to_name:  # If we are the one recieving
            request.other_name = from_name  # Then the other one is the one sending
        elif self.name == from_name:  # We are the one sending
            request.other_name = to_name  # Then the other one is the one recieving
        else:
            '''
                Error (The one calling this is trying to create an invalid transfer - the transfer request must come from the nodes involved )
            '''
            self.get_logger().error(
                "Invalid transfer request: Node %s can't start transfer: %s"
                % (self.name, (from_name + " to " + to_name))
            )
            return self.status["ERROR"]  # Error

    # Wait for service
    load_transfer_cli = self.create_client(
        LoadTransfer, "/arm/%s/load_transfer" % arm_id
    )
    while not load_transfer_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Call client
    status = self.status["WAITING"]
    while status == self.status["WAITING"] :  # 10 is WAITING
        future = load_transfer_cli.call_async(request)
        self.get_logger().info(
            "Requesting transfer from %s to %s Node: %s" % (to_name, from_name, self.name)
        )

        # Waiting for completion
        while future.done() == False:
            time.sleep(1)  # 1 second timeout

        if future.done():
            try:
                response = future.result()

                # Error handling - TODO: if an error occured we want to deal with that separately 
                if (
                    response.status == response.ERROR
                    or response.status == response.FATAL
                ):  # Some error occured
                    raise Exception
            except Exception as e:
                self.get_logger().error("Error occured %r" % (e,)) #TODO: Maybe handle this as error and not as waiting
                status = self.status["WAITING"]   # Retry
            else:
                if response.status == response.SUCCESS:
                    self.get_logger().info(
                        "Successfully transfered from name: %s to name: %s"
                        % (from_name, to_name)
                    )
                    status = self.status["SUCCESS"]
                elif response.status == response.WAITING:
                    self.get_logger().info(
                        "Waiting on transfer from: %s to: %s" % (from_name, to_name)
                    )
                    status = self.status["WAITING"]  # WAITING

        '''
            Note: The time.sleep() used to be 5 seconds, but was decided to change to just a simply time.sleep(0) for thread yielding this allows this to run at a much higher frequency to allow the checks to happen more rapidly.
        '''
        if status == self.status["WAITING"] :  # Waiting
            time.sleep(
                0
            )  # Yields to other threads
        time.sleep(1)  # 1 second timeout

    return self.status["SUCCESS"]

# Middleman function to segway to transfer call in retry function
def _load_transfer(args):
    return load_transfer(
        args[0], args[1], args[2], args[3], args[4]
    )  # self, from_name_or_id, to_name_or_id, item, arm_id


# dud main function
def main_null():
    print("This is not meant to have a main")
