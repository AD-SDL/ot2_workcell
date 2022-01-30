import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from workcell_interfaces.srv import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from random import random
from ot2_workcell_manager_client.retry_api import *
from ot2_workcell_manager_client.register_api import *
from ot2_workcell_manager_client.register_api import _register, _deregister_node
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# TODO: figure out how to integrate arm code


class ArmManager(Node):
    def __init__(self, name):
        super().__init__("Temp" + str(int(random() * 17237534)))

        # Parameters before we register with master
        self.declare_parameter(
            "name", "insert_arm_name_here"
        )  # 2nd arg is default value
        time.sleep(2) # Wait for the launch file to hand in names
        name = self.get_parameter("name").get_parameter_value().string_value
        while name == "temp" or name == "insert_arm_name_here":
            self.get_logger().info("Please enter parameter node name")
            rclpy.spin_once(self) # spin self once for parameter
            name = self.get_parameter("name").get_parameter_value().string_value

        # Node creation
        super().__init__("arm_manager_" + name)  # User specifies name

        # Lock creation
        self.arm_lock = Lock()  # Only one can access arm at a time
        self.state_lock = Lock() # Only one can access the state at a time

        # Queues
        self.transfer_queue = []
        self.completed_queue = []
        self.run_queue = []  # Which transfers are currently running

        # Readabilty
        self.state = {  # TODO maybe a sync with the master
            "BUSY": 1,
            "READY": 0,
            "ERROR": 2,
        }
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3}

        # State information
        self.current_state = self.state["READY"]  # Start ready

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/"

        # Create clients

        # Register with master
        args = []
        args.append(self)  # Self
        args.append("arm")  # Type
        args.append(name)  # Name
        status = retry(
            self, _register, 10, 1, args
        )  # Setups up a retry system for a function, args is empty as we don't want to feed arguments
        if status == self.status["ERROR"] or status == self.status["FATAL"]:
            self.get_logger().fatal("Unable to register with master, exiting...")
            sys.exit(1)  # Can't register node even after retrying

        # Create services
        self.get_id_service = self.create_service(
            GetId, "/arm/%s/get_id" % self.name, self.get_id_handler
        )
        self.load_transfer_service = self.create_service(
            LoadTransfer, "/arm/%s/load_transfer" % self.id, self.load_transfer_handler
        )
        self.get_next_transfer_service = self.create_service(
            GetNextTransfer,
            "/arm/%s/get_next_transfer" % self.id,
            self.get_next_transfer_handler,
        )

        # Create subscribers
        self.arm_state_update_sub = self.create_subscription(
            ArmStateUpdate,
            "/arm/%s/arm_state_update" % self.id,
            self.arm_state_update_callback,
            10,
        )
        self.arm_state_update_sub  # Prevent unused warning
        self.completed_transfer_sub = self.create_subscription(
            CompletedTransfer,
            "/arm/%s/completed_transfer" % self.id,
            self.completed_transfer_callback,
            10,
        )
        self.completed_transfer_sub  # prevent unused warning
        self.state_reset_sub = self.create_subscription(ArmReset, "/arm/%s/arm_state_reset"%self.id,self.state_reset_callback, 10)
        self.state_reset_sub # prevent unused variable warning

        # Initialization Complete
        self.get_logger().info(
            "Arm Manager for ID: %s name: %s initialization completed"
            % (self.id, self.name)
        )

    # Upon a completed transfer this function adds it to the queue
    def completed_transfer_callback(self, msg):
        # Get arm lock
        self.arm_lock.acquire()

        # Get data
        identifier_cur = msg.identifier_cur
        identifier_other = msg.identifier_other

        # Add to completed queue
        self.completed_queue.append(identifier_cur)
        self.completed_queue.append(identifier_other)

        # Remove from transfer
        self.transfer_queue.remove(identifier_cur)
        self.transfer_queue.remove(identifier_other)

        # Remove from run queue (TODO: assertion check, the popped is the same as the completed)
        self.run_queue.pop(0) # The one that was being worked on was the first one

        self.get_logger().info(
            "Completed transfer " + str(self.completed_queue)
        )  # TODO: DELETE

        # Release lock
        self.arm_lock.release()

    # Allows the transfer handler to get the next transfer (service call)
    def get_next_transfer_handler(self, request, response):

        # create response
        response = GetNextTransfer.Response()

        # get state (lock)
        self.state_lock.acquire()

        if(self.current_state == self.state['ERROR']):
            self.get_logger().error("Arm in error state")
            response.status = response.WAITING # Tell it to wait until error is resolved (TODO: switch to error)
            self.state_lock.release() # Release lock
            return response
        elif(self.current_state == self.state['BUSY']): # This should never happen, as it won't call this service until the state is ready
            self.get_logger().error("Arm in busy state")
            response.status = response.WAITING # wait for state not to be busy TODO: switch to error
            self.state_lock.release() # Release lock
            return response

        # release lock
        self.state_lock.release()

        # Get lock
        self.arm_lock.acquire()

        # 		self.get_logger().info("Get next transfer " + str(self.run_queue)) #TODO: DELETE
        # 		self.get_logger().info("Get next transfer " + str(self.transfer_queue)) #TODO: DELETE

        # Retrieve next item in queue
        if len(self.run_queue) > 0:
            response.next_transfer = self.run_queue[0]  # Get the first in the queue (don't remove it, upon completion it is removed)
            response.status = response.SUCCESS
        else:
            response.status = response.WAITING  # Waiting on things to run

        # 		self.get_logger().info("Get next transfer " + str(self.run_queue)) #TODO: DELETE
        # 		self.get_logger().info("Get next transfer " + str(self.transfer_queue)) #TODO: DELETE
        # 		self.get_logger().info("get next transfer done") #TODO: DELETE

        # Release lock
        self.arm_lock.release()

        # Return
        return response

    # handler for the next transfer service call
    def load_transfer_handler(self, request, response):  # TODO: error handling
        # Acquire lock
        self.arm_lock.acquire()

        # Get request
        to_name = request.to_name
        to_id = request.to_id
        from_name = request.from_name
        from_id = request.from_id
        item = request.item
        cur_node = request.cur_name
        other_node = request.other_name

        # Create response
        response = LoadTransfer.Response()

        # Create identifier
        identifier_cur = from_name + " " + to_name + " " + item + " Node: " + cur_node
        identifier_other = (
            from_name + " " + to_name + " " + item + " Node: " + other_node
        )
        # 		self.get_logger().info("cur: %s   other: %s"%(identifier_cur, identifier_other)) #TODO: DELETE

        # Check to see if the transfer already completed
        completed = False
        for item in self.completed_queue:
            if item == identifier_other:  # Transfer already completed
                completed = True
                self.completed_queue.remove(item)  # remove from completed queue
                break
        if completed == True:  # We are done
            response.status = response.SUCCESS
            self.arm_lock.release()  # Realise lock
            return response

        # Check to see if other side is ready
        both_ready = False #TODO: Switch name and test
        for item in self.transfer_queue:
            if item == identifier_other:  # Item is waiting can continue
                both_ready = True
                break

        # Check if in queue
        in_queue = False
        for item in self.transfer_queue:
            if item == identifier_cur:
                in_queue = True
                break  # in queue already

        # Adds current transfer identifier for the other side to verify
        if in_queue == False:
            self.transfer_queue.append(identifier_cur)

        # 		self.get_logger().info("both ready %s, in queue %s"%(str(both_ready), str(in_queue))) #TODO: DELETE
        # If both aren't ready we return WAITING, or if both are listed in transfer (being processed)
        if both_ready == False or (in_queue == True and both_ready == True):
            response.status = response.WAITING  # Still waiting on the other side
            self.arm_lock.release()  # Realise lock
            return response

        # if this point is reached the transfer is ready to be run
        # Add to run queue
        self.run_queue.append(
            identifier_cur + "\n" + identifier_other
        )  # For the node waiting on it
        self.arm_lock.release()  # Release lock
        response.status = (
            response.WAITING
        )  # If it isn't in completed then it isn't done

        # 		self.get_logger().info("Load transfer " + str(self.transfer_queue)) #TODO: DELETE
        # 		self.get_logger().info("Load transfer " + str(self.run_queue)) #TODO: DELETE
        # 		self.get_logger().info("Load transfer " + str(self.completed_queue)) #TODO: DELETE
        return response

    # Function to reset the state of the transfer handler
    def state_reset_callback(self, msg):
        self.get_logger().warning("Resetting state...")

        # Get state lock
        self.state_lock.acquire()

        self.current_state = msg.state

        # Release lock
        self.state_lock.release()

    # Service to update the state of the arm
    def arm_state_update_callback(self, msg):
        # Lock the state
        self.state_lock.acquire()

        # Prevent changing state when in an error state
        if(self.current_state == self.state['ERROR']):
            self.get_logger().error("Can't change state, the state of the arm is already error")
            self.state_lock.release() # release lock
            return # exit out of function

        # Recieve request
        self.current_state = msg.state  # TODO error checks

        # TODO: sync with master

        # 		self.get_logger().info("I Heard %d"%msg.state) # TODO: DELETE
        self.state_lock.release() # release lock


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


def main(args=None):
    rclpy.init(args=args)

    # 	if(len(sys.argv) != 2):
    # 		print("need 1 arguments")
    # 		sys.exit(1)
    # 	name = str(sys.argv[1])
    name = "temp"  # TODO: DELETE

    arm_manager_node = ArmManager(name)
    try:
        rclpy.spin(arm_manager_node)
    except:
        arm_manager_node.get_logger().error("Terminating...")

    # End
    args = []
    args.append(arm_manager_node)
    status = retry(
        arm_manager_node, _deregister_node, 10, 1.5, args
    )  # TODO: handle status
    arm_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
