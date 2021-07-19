import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from workcell_interfaces.srv import *
from pathlib import Path
from ot2_workcell_manager_client.retry_api import *

# Runs a module on the node (id)
def run(self, file, id):
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

    # Get type
    node = self.search_for_node(id)
    type = node["type"]

    # Client setup
    run_cli = self.create_client(
        Run, "/%s/%s/run" % (type, id)
    )  # format of service is /{type}/{id}/{service name}
    while not run_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Create a request
    req = Run.Request()
    req.type = type
    req.id = id
    req.file = file

    # Call service
    future = run_cli.call_async(req)
    self.get_logger().info("Waiting for completion...")

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
                self.get_logger().info("Module run succeeded")
                return self.status["SUCCESS"]  # All good

# Loads filename to a worker node
# parameters, name of file (path not needed done by worker), id of worker, and if the file already exists do we replace it or not?
def load(self, name, id, replacement):
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
        target_node = self.search_for_node(
            id
        )  # See if id robot exists and the data

        # Error checking
        if target_node["type"] == "-1":  # No such node
            self.get_logger().error("id: %s doesn't exist" % id)
            return self.status["ERROR"]

        # 			target_node = self.nodes_list[int(random()*len(self.nodes_list))] # Random load assignment
        type = target_node["type"]  # These will be needed to acess the service
        id = target_node["id"]
    except Exception as e:
        self.get_logger().error("Error occured: %r" % (e,))
        return self.status["ERROR"]

    # Client setup    (client can't be in the class as it constantly changes)
    #
    load_cli = self.create_client(
        LoadService, "/%s/%s/load" % (type, id)
    )  # format of service is /{type}/{id}/{service name}
    while not load_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Client ready
    try:
        f = open(self.module_location + name, "r")
        contents = f.read()
        f.close()
    except Exception as e:
        self.get_logger().error("Error occured: %r" % (e,))
        return self.status["ERROR"]  # Error

    self.get_logger().info("File %s read complete" % name)  # Contents of file read

    # Create a request
    load_request = LoadService.Request()
    load_request.name = name  # File path: insert file name, the file path even though the same is given to the client to set up
    load_request.contents = contents  # File string contents
    load_request.replace = replacement  # If the file exists do we overwrite it?

    # Call service to load module
    future = load_cli.call_async(load_request)
    self.get_logger().info("Waiting for completion...")

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
            # Error handling
            if response.status == response.ERROR:
                self.get_logger().error(
                    "Error occured in loading at %s for file %s" % (id, name)
                )
                return self.status["ERROR"]  # Error
            elif response.status == response.WARNING:
                self.get_logger().warning(
                    "Warning: File %s already exists on system %s" % (name, id)
                )
                return self.status["WARNING"]  # Warning
            else:
                self.get_logger().info("Load succeeded")
                return self.status["SUCCESS"]  # All good

# Function to segway to main function call
def _load(self, args):
    return self.load(args[0], args[1], args[2])  # File, id, replacement

# Function to segway to main function call
def _run(self, args):
    return self.run(args[0], args[1])  # File, id

# Function to just run load and run
def load_and_run(self, file, id):
    # Load the module
    args = []
    args.append(file)
    args.append(id)
    args.append(True)  # Auto update
    status = retry(
    self, self._load, 1, 0, args
    )  # Calling retry function with 1 attempt, just want output information

    # Status check
    if status == self.status["FATAL"]:
        self.get_logger().fatal(
            "Major error occured when attempting to run function"
        )
        return self.status["FATAL"]  # Fatal error
    elif status == self.status["ERROR"]:
        self.get_logger().error(
            "Retry function stopped, either due to too many attempts or a bad status returned"
        )
        return self.status["ERROR"]  # Error
    elif status == self.status["WARNING"]:
        self.get_logger().warning(
            "Warning thrown by retry function during execution, but ran to completion"
        )
        # Continue
    else:
        self.get_logger().info("Function ran to completion")
        # Continue

    # Run the module
    args = []
    args.append(file)
    args.append(id)
    status = retry(
        self, self._run, 1, 0, args
    )  # Calling retry function with 1 attempt to get output messages

    # Status check
    if status == self.status["FATAL"]:
        self.get_logger().fatal(
            "Major error occured when attempting to run function"
        )
        return self.status["FATAL"]  # Fatal error
    elif status == self.status["ERROR"]:
        self.get_logger().error(
            "Retry function stopped, either due to too many attempts or a bad status returned"
        )
        return self.status["ERROR"]  # Error
    elif status == self.status["WARNING"]:
        self.get_logger().warning(
            "Warning thrown by retry function during execution, but ran to completion"
        )
        return self.status["WARNING"]  # Warnning thrown
    else:
        self.get_logger().info("Function ran to completion")
        return self.status["SUCCESS"]  # All Good

def main_null():
    print("This function isn't meant to have a main functino")
