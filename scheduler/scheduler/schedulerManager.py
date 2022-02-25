# ROS Library
import rclpy
from rclpy.node import Node

# Other
from threading import Thread, Lock
import sys
import time
from pathlib import Path
import importlib.util
from random import random

# ROS messages and services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# OT2_workcell_manager API
from ot2_workcell_manager_client.retry_api import *
from ot2_workcell_manager_client.register_api import *
from ot2_workcell_manager_client.register_api import _register, _deregister_node
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# OT2 Control API
from ot2_client.ot2_control_api import load_protocols_to_ot2, add_work_to_ot2

'''
    The schedulerManager node is responsible for scheduling protocols across the OT2s this means it requires state information and needs to interact with the database. 

    TODO: 2 node system like arm and ot2 or 1 node system like master 
'''
class schedulerManager(Node):
    def __init__(self, name):
        ''' TODO: Maybe add this
        super().__init__("Temp" + str(int(random() * 17237534)))

        # Parameters before we register with master
        self.declare_parameter(
            "name", "insert_scheduler_name_here"
        )  # 2nd arg is default value
        time.sleep(2) # Wait for the launch file to hand in names
        name = self.get_parameter("name").get_parameter_value().string_value
        while name == "temp" or name == "insert_scheduler_name_here":
            self.get_logger().info("Please enter parameter node name")
            rclpy.spin_once(self) # spin self once for parameter
            name = self.get_parameter("name").get_parameter_value().string_value
        '''
        # Node creation
        super().__init__("scheduler_manager_" + name)  # User specifies name


        # Readabilty
        self.state = {  # TODO: maybe a sync with the master
            "BUSY": 1,
            "READY": 0,
            "ERROR": 2,
        }
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

        # Queues 
        self.protocol_queue = [] # protocols to run 

        # State information
        self.current_state = self.state["READY"]  # Start ready

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/" #TODO: this might change 

        # Register with master
        args = []
        args.append(self)  # Self
        args.append("scheduler")  # Type
        args.append(name)  # Name
        status = retry(
            self, _register, 1000, 1, args
        )  # Setups up a retry system for a function
        if status == self.status["ERROR"] or status == self.status["FATAL"]:
            self.get_logger().fatal("Unable to register with master, exiting...")
            sys.exit(1)  # Can't register node even after retrying

        # TODO: services and topics
        self.get_id_service = self.create_service( # Service to add work to the queue 
            SchedulerWork, "/scheduler/%s/AddWork" % self.id, self.add_work_handler #TODO: SchedulerWork service type has response of string[] 
        )

        # Initialization Complete
        self.get_logger().info(
            "Scheduler Manager for ID: %s name: %s initialization completed"
            % (self.id, self.name)
        )

    '''
        This reads from a setup file in the OT2_Modules directory which contains the work for each robot that needs to be 
        run. Currently it is possible for the system to deadlock due to circular wait with the transfer requests, since 
        both robots need to be ready for the arm (Technically the OT2 are the resource as it waits on the other robot). 
        This could cause issues that need to be addressed in the future. 
    '''
    def read_from_setup(self, file):  # TODO: deadlock detection algorithm
        # Read from setup file and distrubute to worker threads - Read number of threads
        f = open(
            self.module_location + file, "r"
        )  # Open up file "setup" in well-known directory
        n = int(
            f.readline()
        )  # First line should contain an integer, corresponds to number of threads

        # For each thread
        for i in range(n):  # Starts reading names and files to be run
            # Get identification
            name_or_id = f.readline().strip()  # Remove newline

            # Find entry for that id or name (spin to wait for it)
            entry  = get_node_info(self, name_or_id)
            if entry['type'] == -1:
                self.get_logger().error(
                    "Unable to find node %s" % name_or_id
                )  # Node isn't registered
                return self.status["ERROR"]
            else:
                id = entry["id"]
                self.get_logger().info("Node %s found" % name_or_id)  # Found

            # Get files for the worker
            try:
                files = f.readline()
            except Exception as e:
                self.get_logger().error("Reading from setup error: %r" % (e,))
                return self.status["ERROR"]  # Error

            split_files = files.split()

            # TODO: Maybe parallelize this part of the program
            # files get split and have their contents sent one by one to OT-2 controller
            for i in range(len(split_files)):
                if(not split_files[i].split(":")[0] == 'transfer'): # Don't send files if transfer
                    load_protocols_to_ot2(self, entry, split_files[i])

            # files sent to worker OT-2 to become threads
            add_work_to_ot2(self, entry, files)

            # Setup complete for this thread
            self.get_logger().info("Setup complete for %s" % name_or_id)

        self.get_logger().info("Setup file read and run complete")
        return self.status["SUCCESS"]

    '''
        This service handler adds the request work to the manager queue to be scheduled
    '''
    def add_work_handler(self, request, response): 
        pass

def main(args=None):
    rclpy.init(args=args)

    scheduler_manager_node = schedulerManager("ana")
    try:
        # Create a thread to run setup_thread
        spin_thread = Thread(target=scheduler_manager_node.read_from_setup, args=("setup",)) #TODO: make it so you can press a button to start it
        spin_thread.start()

        rclpy.spin(scheduler_manager_node)
    except:
        scheduler_manager_node.get_logger().error("Terminating...")

    # End
    spin_thread.join()
    args = []
    args.append(scheduler_manager_node)
    status = retry(scheduler_manager_node, _deregister_node, 10, 1.5, args)  # TODO: handle status
    scheduler_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
