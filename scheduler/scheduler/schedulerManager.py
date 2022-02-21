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

        # Initialization Complete
        self.get_logger().info(
            "Scheduler Manager for ID: %s name: %s initialization completed"
            % (self.id, self.name)
        )


def main(args=None):
    rclpy.init(args=args)

    scheduler_manager_node = schedulerManager("ana")
    try:
        rclpy.spin(scheduler_manager_node)
    except:
        scheduler_manager_node.get_logger().error("Terminating...")

    # End
    args = []
    args.append(scheduler_manager_node)
    status = retry(scheduler_manager_node, _deregister_node, 10, 1.5, args)  # TODO: handle status
    scheduler_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
