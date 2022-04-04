# ROS Libraries
from importlib.metadata import entry_points
import rclpy
from rclpy.node import Node

# Time Library
import time

# ROS messages and services 
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# Retry API
from ot2_workcell_manager_client.retry_api import *

# Transfer api import
from arm_client.transfer_api import _load_transfer

# Database 
from database.database_functions import *
from database.database_functions import insert_protocol
from database.protocol_parser import *
from database.protocol_parser import protocol_parser

    
'''
    Input: ROS object,  node entry, Single string with the protocol name
    Output: Status signal (self.status)

    These functions will load a protocol file to the respective OT2 and provide the ability to add a protocol to the run queue of the OT2. 
    Creates client that sends contents of files to OT-2
    TODO: eliminate the need for master (switch to get_node_info)
'''
def load_protocols_to_ot2(self, entry, name):

    # Check node online?
    '''
    args = []
    args.append(id)
    status = retry(
        self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args
    )  # retry function
    if status == self.status["ERROR"] or status == self.status["FATAL"]:
        self.get_logger().error(
            "Unable to find node %s" % id
        )  # Node isn't registered
        return self.status["ERROR"]
    else:
        self.get_logger().info("Node %s found" % id)  # Found
    '''

    # Select a node
    id = -1 # Init
    try:
        # Get node information
        #target_node = self.search_for_node(id)  # See if id robot exists
        target_node = entry

        # Error checking
        if target_node["type"] == "-1":  # No such node
            self.get_logger().error("id: %s doesn't exist" % id)
            return self.status["ERROR"]

        type = target_node["type"]  # These will be needed to acess the service
        id = target_node["id"]

    except Exception as e:
        self.get_logger().error("Error occured: %r" % (e,))
        return self.status["ERROR"]

    # insert error handling 
    protocol_new_name = protocol_parser(self.module_location + name)

    # insert protocol into database 
    protocol_id = insert_protocol(self.module_location + protocol_new_name, target_node['id'])

    # return id 
    return protocol_id

'''
    Input: ROS object,  node entry, list of strings (protocols)
    Output: status signal (self.status)

    Creates client that sends files to worker OT-2 to create threads
'''
def add_work_to_ot2(self, entry, protocol_id_list, block_name):  # self, id of robot, protocol ids or transfer command, and block name

    # Check node online?
    '''
    args = []
    args.append(id)
    status = retry(
        self, self.node_ready, self.node_wait_attempts, self.node_wait_timeout, args
    )  # retry function
    if status == self.status["ERROR"] or status == self.status["FATAL"]:
        self.get_logger().error(
            "Unable to find node %s" % id
        )  # Node isn't registered
        return self.status["ERROR"]
    else:
        self.get_logger().info("Node %s found" % id)  # Found
    '''

    # Select a node
    id = -1 # Init 
    try:
        # Get node information
        #target_node = self.search_for_node(id)  # See if id robot exists
        target_node = entry

        # Error checking
        if target_node["type"] == "-1":  # No such node
            self.get_logger().error("id: %s doesn't exist" % id)
            return self.status["ERROR"]

        type = target_node["type"]  # These will be needed to access the service
        id = target_node["id"]

    except Exception as e:
        self.get_logger().error("Error occured: %r" % (e,))
        return self.status["ERROR"]

    # create client that calls file handler service on OT-2 module

    # Client setup
    send_cli = self.create_client(
        AddWork, "/%s/%s/add_work" % (type, id)
    )  # format of service is /{type}/{id}/{service name}
    while not send_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Client ready
    # TODO: replacement parameter?
    
    # Create a request
    send_request = AddWork.Request()
    # send_request.numFiles = len(files) # number of files to be sent to worker node
    send_request.protocol_id_list = protocol_id_list  # string of file names list
    send_request.block_name = block_name # block name

    # Call Service to load module
    future = send_cli.call_async(send_request)

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
                    "Error occured when running file %s at id: %s" % (name, id)
                )
                return self.status["ERROR"]  # Error
            else:
                self.get_logger().info("Work added to OT2 queue for: %s"%(entry['name']))
                return self.status["SUCCESS"]  # All good