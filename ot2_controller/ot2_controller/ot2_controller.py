import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from ot2_workcell_manager_client.retry_api import *
from ot2_workcell_manager_client.register_api import *
from ot2_workcell_manager_client.register_api import _register, _deregister_node
from arm_client.transfer_api import *
from arm_client.transfer_api import _load_transfer
from random import random

class OT2(Node):
    def __init__(self, name):
        super().__init__("Temp" + str(int(random() * 17237534)))

        # Parameters before we register with master
        self.declare_parameter(
            "name", "insert_ot2_name_here"
        )  # 2nd arg is default value
        time.sleep(2) # Wait for the launch file to hand in names
        name = self.get_parameter("name").get_parameter_value().string_value
        while name == "temp" or name == "insert_ot2_name_here":
            self.get_logger().info("Please enter name parameter")
            rclpy.spin_once(self)
            name = self.get_parameter("name").get_parameter_value().string_value

        # Node creation
        super().__init__("ot2_" + name)  # Users specifies name

        # Lock creation
        self.file_lock = Lock()  # Access to the file system
        self.run_lock = Lock()  # Only one can access this OT-2 as a resource
        self.work_list_lock = Lock() # Access to the work list
        self.state_lock = Lock() # Access to the state

        # readability
        self.state = {"BUSY": 1, "READY": 0, "ERROR": 2}  # TODO: sync with master
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3}

        # State information
        self.current_state = self.state["READY"]

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ros2tests/OT2_Modules/"

        self.work_list = []  # list of files list recieved from jobs
#        self.threads_list = []  # list of all worker threads

        # Node timeout info
        self.node_wait_timeout = 2  # 2 seconds
        self.node_wait_attempts = 10  # 10 attempts before error thrown

        # Create clients
        # 		self.register_cli = self.create_client(Register, 'register') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)
        # 		self.deregister_cli = self.create_client(Destroy, 'destroy') # All master service calls will be plain, not /{type}/{id} (TODO: change to this maybe?)

        # Register with master
        args = []
        args.append(self)  # Self
        args.append("OT_2")  # Type
        args.append(name)  # Name
        status = retry(
            self, _register, 10, 1, args
        )  # Setups up a retry system for a function, args is empty as we don't want to feed arguments
        if status == self.status["ERROR"] or status == self.status["FATAL"]:
            self.get_logger().fatal("Unable to register with master, exiting...")
            sys.exit(1)  # Can't register node even after retrying

        # Create services: Have to wait until after registration this way we will have the id
        self.get_id_service = self.create_service(
            GetId, "/OT_2/%s/get_id" % self.name, self.get_id_handler
        )
        self.send_service = self.create_service(
            SendFiles, "/OT_2/%s/send_files" % self.id, self.receive_files_handler
        )
        self.protocol_service = self.create_service(
            Protocol, "/OT_2/%s/protocol" % self.id, self.protocol_handler
        )

        # Create subscribers
        self.ot2_state_update_sub = self.create_subscription(
            OT2StateUpdate,
            "/OT_2/%s/ot2_state_update" % self.id,
            self.ot2_state_update_callback,
            10,
        )
        self.ot2_state_update_sub  # prevent unused warning
        self.state_reset_sub = self.create_subscription(OT2Reset, "/OT_2/%s/ot2_state_reset"%self.id,self.state_reset_callback, 10)
        self.state_reset_sub # prevent unused variable warning

        # TODO: create service to unload and recieve items

        # Initialization Complete
        self.get_logger().info(
            "ID: %s name: %s initialization completed" % (self.id, self.name)
        )

    # Handles send_module service calls
    def receive_files_handler(self, request, response):

        # Get request information
        files = request.files
        files = files.split()

        # Create Response
        response = SendFiles.Response()

        # Begin reading file names
        self.get_logger().info("Reading file names")

        try:
            # Get lock
            self.work_list_lock.acquire()

            # Append files to work list
            for item in files:
                self.work_list.append(item)
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            response.status = response.ERROR  # Error
        else:
            self.get_logger().info("Files %s loaded to OT2" % files)
            response.status = response.SUCCESS  # All good
        finally:
            # Exiting critical section
            self.work_list_lock.release()
            return response

    # Function to reset the state of the transfer handler
    def state_reset_callback(self, msg):
        self.get_logger().warning("Resetting state...")

        # Get state lock
        self.state_lock.acquire()

        self.current_state = msg.state

        # Release lock
        self.state_lock.release()

    # Service to update the state of the ot2
    def ot2_state_update_callback(self, msg):

        # Recieve request
        current_state = msg.state  # TODO error checks

        # 		self.get_logger().info("I Heard %d"%msg.state) # TODO: DELETE

        # Update our state
        self.current_state = current_state

        # TODO: other stuff

    # Service to retrieve ID of the robot
    def get_id_handler(self, request, response):
        # Retrieve id and node information
        id = self.id
        name = self.name
        type = self.type

        # create response
        response = GetId.Response()
        response.id = id
        response.name = name
        response.type = type

        # Return response
        return response

    # handles protocol module service calls
    def protocol_handler(self, request, response):

        # get state lock
        self.state_lock.acquire()

        if(self.current_state == self.state['ERROR']):
            self.get_logger().error("OT2 in error state")
            response.status = response.WAITING # Tell it to wait until error is resolved (TODO: switch to error)
            self.state_lock.release() # Release lock
            return response
        elif(self.current_state == self.state['BUSY']): # This should never happen, as it won't call this service until the state is ready
            self.get_logger().error("OT2 in busy state")
            response.status = response.WAITING # wait for state not to be busy TODO: switch to error
            self.state_lock.release() # Release lock
            return response

        # release lock
        self.state_lock.release()

        # No request information

        # Create response
        response = Protocol.Response()

        # Warnings / Errors
        if not id == self.id:  # Wrong ID
            self.get_logger().error(
                "Request id: %s doesn't match node id: %s" % (id, self.id)
            )
            response.status = response.ERROR
            return response
        elif not type == "OT_2":  # Wrong type
            self.get_logger().warning(
                "The requested node type: %s doesn't match the node type of id: %s, but will still proceed"
                % (type, self.id)
            )
        elif path.exists(file) == False:  # File doesn't exist
            self.get_logger().error("File: %s doesn't exist" % (file))
            response.status = response.ERROR
            return response

        # Get lock
        self.work_list_lock.acquire()

        # obtain file containing protocol
        self.get_logger().info("Handing over file")

        try:
            # Extract file name from list
            name = self.work_list.pop(0)
            response.file = name

            # clear work_list
#            self.work_list.clear()
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            response.status = response.ERROR  # Error
        else:
            self.get_logger().info("File %s handed to OT2" % name)
            response.status = response.SUCCESS  # All good

        finally:
            # Exiting critical section
            self.work_list_lock.release()
            return response

# TODO: DELETE
def work(ot2node):

    args = []
    if ot2node.name == "bob":
        args.append(ot2node)
        args.append("bob")
        args.append("alex")
        args.append("10")
        args.append("army")
        status = retry(ot2node, _load_transfer, 20, 4, args)
    if ot2node.name == "alex":
        args.append(ot2node)
        args.append("bob")
        args.append("alex")
        args.append("10")
        args.append("army")
        status = retry(ot2node, _load_transfer, 20, 4, args)

def main(args=None):
    rclpy.init(args=args)

    name = "temp" #TODO: delete

    ot2node = OT2(name)

    ot2node.get_logger().info("init done") 
    # Spin
    try:
        # TODO: DELETE
        spin_thread = Thread(
            target=work,
            args=(
                ot2node,
            ),
        )
        spin_thread.start()

        rclpy.spin(ot2node)
    except:
        ot2node.get_logger().error("Terminating...")

        # Setup args and end
        args = []
        args.append(ot2node)  # Self
        status = retry(ot2node, _deregister_node, 10, 1.5, args)  # TODO: handle status
        spin_thread.join()
        ot2node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
