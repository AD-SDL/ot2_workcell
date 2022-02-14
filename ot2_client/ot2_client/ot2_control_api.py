    
import rclpy
from rclpy.node import Node
import time
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
from ot2_workcell_manager_client.retry_api import *

# Transfer api import
from arm_client.transfer_api import *
from arm_client.transfer_api import _load_transfer
    
'''
    Note: These functions are only currently usable by the master node as there is no search_for_node function in the OT2 package or Arm Package, therefore the API would cause an error
    since that function is not found. 

    TODO: Move the search_for_node function into an API provided by the master. (get_node_info part of the worker_api)

    TODO: element the need for master (switch to get_node_info)
'''

# Creates client that sends contents of files to OT-2
def load_protocols_to_ot2(self, id, name):

    # Check node online?
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

    # Select a node
    try:
        # Get node information
        target_node = self.search_for_node(id)  # See if id robot exists

        # Error checking
        if target_node["type"] == "-1":  # No such node
            self.get_logger().error("id: %s doesn't exist" % id)
            return self.status["ERROR"]

        type = target_node["type"]  # These will be needed to acess the service
        id = target_node["id"]

    except Exception as e:
        self.get_logger().error("Error occured: %r" % (e,))
        return self.status["ERROR"]
    

    # Create client that calls load_protocols servive on controller

    script_cli = self.create_client(LoadProtocols, "/%s/%s/load_protocols" % (type, id))  # format of service is /{type}/{id}/{service name}
    while not script_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # extract name and contents of each first file in list
    with open(self.module_location + name, 'r') as file:
        contents = file.read()
    

    # Client ready
    script_request = LoadProtocols.Request()
    script_request.name = name # name of file
    script_request.contents = contents # contents of file
    script_request.replace = True # Replace file of same name (default true)

    # Call service
    future = script_cli.call_async(script_request)

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
                    "Error occured when sending script %s at id: %s" % (name, id)
                )
                return self.status["ERROR"]  # Error
            else:
                self.get_logger().info("File %s contents loaded" % name)
                return self.status["SUCCESS"]  # All good

# Creates client that sends files to worker OT-2 to create threads
def add_work_to_ot2(self, id, files):  # self, id of robot, and files of current job

    # Check node online?
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

    # Select a node
    try:
        # Get node information
        target_node = self.search_for_node(id)  # See if id robot exists

        # Error checking
        if target_node["type"] == "-1":  # No such node
            self.get_logger().error("id: %s doesn't exist" % id)
            return self.status["ERROR"]

        type = target_node["type"]  # These will be needed to acess the service
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
    send_request.files = files  # string of file names list

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
                self.get_logger().info("Files loaded")
                return self.status["SUCCESS"]  # All good