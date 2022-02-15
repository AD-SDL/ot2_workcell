# ROS library
import rclpy
from rclpy.node import Node

from threading import Thread, Lock
import time
import sys

# ROS messages and services 
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# ot2_workcell_manager library 
from ot2_workcell_manager_client.retry_api import *
from ot2_workcell_manager_client.register_api import *
from ot2_workcell_manager_client.register_api import _get_id_name
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# Arm Libraries 
from arm_client.transfer_api import *
from arm_client.transfer_api import _load_transfer

# ot2_client libraries 
from ot2_client.publish_ot2_state_api import *
from ot2_client.publish_ot2_state_api import _update_ot2_state

# Others 
from pathlib import Path
import importlib.util
from random import random
from typing import Protocol


class OT2ProtocolManager(Node):
    def __init__(self, name):
        # Create a temporary node so we can read in parameters
        super().__init__("Temp" + str(int(random() * 17237967)))

        # Create parameters for name to be sent through
        self.declare_parameter(
            "name", "insert_OT2_protocol_manager_name_here"
        )  # 2nd arg is default value
        time.sleep(2) # Wait for the launch file to hand in names
        name = self.get_parameter("name").get_parameter_value().string_value
        while name == "temp" or name == "insert_OT2_protocol_manager_name_here":
            name = self.get_parameter("name").get_parameter_value().string_value
            rclpy.spin_once(self)
            self.get_logger().info("Please enter the name parameter to this node")

        # Node creation
        super().__init__("OT2_protocol_manager_" + name)  # User specifies name
        self.name = name
        self.type = "OT_2"

        # Lock creation
        self.run_lock = Lock()  # Only one can access ot2 at a time

        # Readabilty
        self.state = {  # TODO maybe a sync with the master
            "BUSY": 1,
            "READY": 0,
            "ERROR": 2,
        }
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

        # State of the ot2
        self.current_state = self.state["READY"]

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/"

        # Create clients

        # Get ID and confirm name from manager
        args = []
        args.append(self)
        status = retry(self, _get_id_name, 5, 2, args)  # 5 retries 2 second timeout
        if status == self.status["ERROR"]:
            self.get_logger().fatal("Unable to get id from ot2 manager, exiting...")
            sys.exit(1)  # TODO: alert manager of error

        # Create services

        # Create subs
        self.state_reset_sub = self.create_subscription(
            OT2Reset,
            "/OT_2/%s/ot2_state_reset"%self.id,
            self.state_reset_callback,
            10,
        )
        self.state_reset_sub # prevent unused variable warning

        # Initialization Complete
        self.get_logger().info(
            "OT2 protocol manager for ID: %s name: %s initialization completed"
            % (self.id, self.name)
        )

    # retrieves next script to run in the queue
    def get_next_protocol(self):

        # Set node info
        type = "OT_2"
        id = self.id

        # Create client
        protocol_cli = self.create_client(Protocol, "/%s/%s/protocol" % (type, id)) # format of service is /{type}/{id}/{service name}
        while not protocol_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available, trying again...")

        # Get the next protocol
        file_name = ""

        # Client ready, get name of file - No request info needed
        protocol_request = Protocol.Request()

        # Call service to protocol
        future = protocol_cli.call_async(protocol_request)
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
                # Get file name
                file_name = response.file
                # Error handling
                if response.status == response.ERROR:
                    self.get_logger().error(
                        "Error occured in protocol client %s for file %s" % (id, response.file)
                    )
                    return self.status["ERROR"]  # Error
                elif response.status == response.WAITING: # Waiting for jobs
                    return self.status['WAITING']
                elif response.status == response.WARNING:
                    self.get_logger().warning(
                        "Warning: File %s already exists on system %s" % (response.file, id)
                    )
                    return self.status["WARNING"]  # Warning
                else:
                    self.get_logger().info("Load succeeded")
                    self.status["SUCCESS"]  # All good

        # Begin running the next protocol
        # TODO: actually incorporate runs

        # set state to busy
        try:
            self.current_state = self.state["BUSY"]
            self.set_state()

            # Conducting a transfer
            if(file_name.split(":")[0] == "transfer"):
                temp = file_name.split(":")
                status = self.transfer(temp[1], temp[2], temp[3], temp[4]) #from, to , item arm
            # run protocol, use load_and_run function
            else:
                self.get_logger().info("Running protocol")
                status = self.load_and_run(file_name)

            # Check to see if run was success
            if status == self.status['ERROR']:
                self.get_logger().error("Error: protocol %s was not run successfully" % file_name)
                raise Exception
            elif status == self.status['SUCCESS']:
                self.get_logger().info("Protocol %s was run successfully" % file_name)
        except Exception as e:
            self.get_logger().error("Error occured: %r"%(e,))
            self.current_state = self.state["ERROR"]
            return self.status['ERROR']
        else:
            self.current_state = self.state["READY"]
            return self.status['SUCCESS']
        finally:
            self.set_state()


    # Function to reset the state of the transfer handler
    def state_reset_callback(self, msg): #TODO: More comprehensive state reset handler 
        self.get_logger().warning("Resetting state...")
        self.current_state = msg.state

    # Takes filename, unpacks script from file, and runs the script
    def load_and_run(self, file):

        # Set variable with file path
        filepath = (self.module_location + file)

        # Error check
        if path.exists(filepath) == False:  # File doesn't exist
            self.get_logger().error("File: %s doesn't exist" % (self.temp_list[0]))
            return self.status['ERROR']

        # Import file as module
        self.get_logger().info("Importing module...")

        try:
            # Load and attach module to program
            spec = importlib.util.spec_from_file_location(file, filepath)
            ot2Module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ot2Module)
        except Exception as e:
            # Error
            self.get_logger().error("Error occured when trying to load module %s: %r" % (filepath, e,))
            return self.status['ERROR']
        else:
            self.get_logger().info("Module %s successfully loaded and attached to the program" % filepath)

        # Run work() function in module
        self.get_logger().info("Running module...")
        try:
            ot2Module.work()
        except Exception as e:
            # Error
            self.get_logger().error("Error occured when trying to run module %s: %r" % (filepath, e))
            return self.status['ERROR']
        else:
            self.get_logger().info("Module %s successfully ran to completion" % filepath)
            return self.status['SUCCESS']

    # Helper function
    def set_state(self):
        args = []
        args.append(self)
        args.append(self.current_state)
        status = retry(self, _update_ot2_state, 10, 2, args)
        if status == self.status["ERROR"] or status == self.status["FATAL"]:
            self.get_logger().error(
                "Unable to update state with manager, continuing but the state of the ot2 may be incorrect"
            )

    # Function to constantly poll manager queue for work, TODO: optimization  - steal work
    def run(self):
        # Run get_protocol every 3 seconds
        while rclpy.ok():
            time.sleep(3)
            status = self.get_next_protocol()  # Full finish before waiting


    # Function to setup transfer
    def transfer(self, from_name, to_name, item, arm_name):
        args = []
        args.append(self)
        args.append(from_name)
        args.append(to_name)
        args.append(item)
        args.append(arm_name)
        status = retry(self, _load_transfer, 20, 4, args)
        return status

def main(args=None):
    rclpy.init(args=args)
    
    protocol_manager = OT2ProtocolManager("temp")
    try:
        # Work
        spin_thread = Thread(target=protocol_manager.run, args=())
        spin_thread.start()

        rclpy.spin(protocol_manager)
    except Exception as e:
        protocol_manager.get_logger().fatal("Error %r" % (e,))
    except:
        protocol_manager.get_logger().error("Terminating...")

    # End
    protocol_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
