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
        self.work_index = 0  # location of recently added files in work_list
        self.threads_list = []  # list of all worker threads
        self.temp_list = [] # List that stores individual jobs for the protocol handler

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
        self.script_service = self.create_service(
            SendScripts, "/OT_2/%s/send_scripts" % self.id, self.load_scripts_handler
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
    
    # Handles send_scripts service calls, creates files and loads contents into them
    def load_scripts_handler(self, request, response):

        # Get request information
        name = request.name # name of file to be created
        contents = request.contents # contents of file
        replace = request.replace # bool whether or not to replace file of same name

        # Create response
        response = SendScripts.Response()

        # Create file
        self.get_logger().info("Creating file %s" % name)

        try:
            # Get lock
            self.file_lock.acquire()

            # TODO: Create file
            # Check if file already exists
            filepath = Path(self.module_location + name)

            if filepath.is_file(): # file already exists
                if replace == True: # request says to replace file
                    os.remove(self.module_location + name) # delete preexisting file
                    f = open(self.module_location + name, "x") # create new file with name
                    f.write(contents) # write contents into file
                    f.close()
                    self.get_logger().info("File %s created and loaded" % name)
                else: # request says don't replce file
                    self.get_logger().info("File already exists, did not replace")
            else: #file does not exist
                f = open(self.module_location + name, "x") # create new file with name
                f.write(contents) # write contents into file
                f.close()
                self.get_logger.info("File %s created and loaded" % name)


        except Exception as e:
            self.get_logger().error("Error occurred: %r" % (e,))
            response.status = response.ERROR
        else:
            self.get_logger().info("File %s created and loaded" % name)
            response.status = response.SUCCESS
        finally:
            # release lock
            self.file_lock.release()
            return response



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
            self.work_list.append(files)
            self.work_index = self.work_index + 1 # Counts total number of jobs given to this OT-2
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

        # Get lock
        self.work_list_lock.acquire()

        # Check to see if work list is empty
        if len(self.temp_list) != 0:
            self.get_logger().info("More work in current job") # Still more files to run in temp_list
        elif len(self.work_list) == 0 and len(self.temp_list) == 0: # Both temp_list and work_list empty, wait for more jobs
            self.get_logger().info("No more current work for OT-2 %s" % self.name)
            response = Protocol.Response()
            response.status = response.SUCCESS
            return response
        else:
            # Selecting job
            self.temp_list = self.work_list[0] #Adds new job (set of files) to the temp_list
            # Remove entry from work list
            self.work_list.pop(0)

        # Check state of OT-2, wait for READY state
        if self.current_state == 1:
            time.sleep(5) # Protocol running, wait 5 seconds
        elif self.current_state == 0:
            self.get_logger().info("OT-2 ready for new protocol")
        elif self.current_state == 2: #Error
            self.get_logger().error("OT-2 in error state")
            response = Protocol.Response()
            response.status = response.ERROR
            return response
        else:
            self.get_logger().error("Error: unexpected state: %s" % self.current_state)

        # Hand over temp_list[0], wai for completion, then delete file

        # Create response
        response = Protocol.Response()

        # Error check
        if path.exists(self.module_location + self.temp_list[0]) == False:  # File doesn't exist
            self.get_logger().error("File: %s doesn't exist" % (self.temp_list[0]))
            response.status = response.ERROR
            return response

        # obtain file containing protocol
        self.get_logger().info("Handing over file")

        try:
            # Extract file name from temp list
            name = self.temp_list[0]
            response.file = name
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            response.status = response.ERROR  # Error
            return response
        else:
            self.get_logger().info("File %s handed to OT2" % name)
            response.status = response.SUCCESS  # All good
        finally:
            # Exiting critical section
            self.work_list_lock.release()

        # Clear temp list
        self.temp_list.pop(0)
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
    try:
        rclpy.spin(ot2node)
    except:
        ot2node.get_logger().fatal("Terminating...")

    # Setup args and end
    args = []
    args.append(ot2node)  # Self
    status = retry(ot2node, _deregister_node, 10, 1.5, args)  # TODO: handle status
#    spin_thread.join()
    ot2node.destroy_node()
    rclpy.shutdown()

'''
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
'''

if __name__ == "__main__":
    main()
