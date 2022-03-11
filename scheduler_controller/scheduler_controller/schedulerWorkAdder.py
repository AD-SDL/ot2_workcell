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
from ot2_workcell_manager_client.register_api import _get_id_name

# scheduler_client 
from scheduler_client.add_blocks_scheduler import add_blocks_scheduler

'''
    This will add read workflow files and send the blocks over to the scheduler to schedule and break down 

    Due to this it doesn't need to register with the master as we should only have on scheduler I think and it only needs to send to that scheduler. 
'''
class schedulerWorkAdder(Node): 
    def __init__(self, name):
        # node creation
        super().__init__("scheduler_work_adder_" + name)

        # Readabilty
        self.state = {  # TODO: maybe a sync with the master
            "BUSY": 1,
            "READY": 0,
            "ERROR": 2,
            "QUEUED": 3,
        }
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/" #TODO: this might change 

        # Initialization Complete
        self.get_logger().info(
            "Scheduler Work Adder name: %s initialization completed"
            % (self.name)
        )

    def submitter(self):
        # get input 
        setup_file_name = input("Setup file name in OT2_modules")

        # Open setup file
        try:
            f = open(
                self.module_location + setup_file_name, "r"
            )  # Open up file "setup" in well-known directory
        except Exception as e: 
            self.get_logger().error("Error occured: %r"%(e,))
            return self.status['ERROR']

        # First line should contain an integer, corresponds to number of threads
        n = int(f.readline())  

        # Load each block 
        blocks = [] # string[]
        for i in range(n):  # Starts reading names and files to be run

            # Get block for the worker
            try:
                block = f.readline()
                blocks.append(block)
            except Exception as e:
                self.get_logger().error("Reading from setup error: %r" % (e,))
                return self.status["ERROR"]  # Error

        # send blocks to scheduler
        status = add_blocks_scheduler(self, blocks)

        # Error handling
        if(status == self.status['ERROR']):
            self.get_logger().error("Setup file read for %s failed"%(setup_file_name,))
            return self.status['ERROR']
        else:
            self.get_logger().info("Setup file: %s read completed"%(setup_file_name,))
            return self.status["SUCCESS"]

def main(args=None):
    rclpy.init(args=args)
    scheduler_work_adder  = schedulerWorkAdder("pigeon")

    try:
        while(True):
            status = scheduler_work_adder.submitter()

            # Error handling
            if(status == scheduler_work_adder.status['ERROR']):
                raise Exception("Error occured in submitter for setup file reads")
    except Exception as e: 
        self.get_logger().error("Error occured: %r"%(e,))
    except:
        self.get_logger().error("Terminating...")

    scheduler_work_adder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()