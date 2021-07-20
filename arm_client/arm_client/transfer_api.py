import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import time
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
from ot2_workcell_manager_client.worker_info_api import *
from ot2_workcell_manager_client.worker_info_api import (
    _get_node_info,
    _get_node_list,
    get_node_info,
)

# TODO: DELETE
def load_transfer(
    self, from_name_or_id, to_name_or_id, item, arm_name_or_id
):  # TODO do something with item

    self.get_logger().info("Running")

    # Get destination node info
    to_entry = get_node_info(self, to_name_or_id)
    from_entry = get_node_info(self, from_name_or_id)

    # Get arm info
    arm_entry = get_node_info(self, arm_name_or_id)

    # Error handling
    if to_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % to_name_or_id)
        return self.status["ERROR"]
    if from_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % from_name_or_id)
        return self.status["ERROR"]
    if arm_entry["type"] == "-1":  # Doesn't exist
        self.get_logger().error("node: %s doesn't exist with master" % arm_name_or_id)
        return self.status["ERROR"]

    # Get name and id
    to_name = to_entry["name"]
    to_id = to_entry["id"]
    from_name = from_entry["name"]
    from_id = from_entry["id"]
    arm_name = arm_entry["name"]
    arm_id = arm_entry["id"]

    # Create request
    request = LoadTransfer.Request()
    request.from_id = from_id
    request.from_name = from_name
    request.to_id = to_id
    request.to_name = to_name
    request.item = item
    if self.type == "master":
        # Bypass restrictions on who does what transfer
        request.cur_name = "master"  # Only the master can remove it from the queue
        request.other_name = "master"
    else:
        request.cur_name = self.name
        if self.name == to_name:  # If we are the one recieving
            request.other_name = from_name  # Then the other one is the one sending
        elif self.name == from_name:  # We are the one sending
            request.other_name = to_name  # Then the other one is the one recieving
        else:  # Error (The one calling this is trying to create an invalid transfer)
            self.get_logger().error(
                "Invalid transfer request: Node %s can't start transfer: %s"
                % (self.name, (from_name + " to " + to_name))
            )
            return self.status["ERROR"]  # Error

    # Wait for service
    load_transfer_cli = self.create_client(
        LoadTransfer, "/arm/%s/load_transfer" % arm_id
    )
    while not load_transfer_cli.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Service not available, trying again...")

    # Call client
    status = 10
    while status == 10:  # 10 is WAITING
        future = load_transfer_cli.call_async(request)
        self.get_logger().info(
            "Requesting a transfer / Adding a entry to transfer queue"
        )

        # Waiting for completion
        while future.done() == False:
            time.sleep(1)  # 1 second timeout

        if future.done():
            try:
                response = future.result()

                # Error handling
                if (
                    response.status == response.ERROR
                    or response.status == response.FATAL
                ):  # Some error occured
                    raise Exception
            except Exception as e:
                self.get_logger().error("Error occured %r" % (e,))
                status = 10  # Retry
            else:
                if response.status == response.SUCCESS:
                    self.get_logger().info(
                        "Successfully transfered from name: %s to name: %s"
                        % (from_name, to_name)
                    )
                    status = self.status["SUCCESS"]
                elif response.status == response.WAITING:
                    self.get_logger().info(
                        "Waiting on transfer from: %s to: %s" % (from_name, to_name)
                    )
                    status = 10  # WAITING

        if status == 10:  # Waiting
            time.sleep(
                5
            )  # Sleep an additional 5 seconds (all other threads to gain access)
        time.sleep(1)  # 1 second timeout

    return self.status["SUCCESS"]


# TODO: DELETE
# Middleman function to segway to transfer call in retry function
def _load_transfer(args):
    return load_transfer(
        args[0], args[1], args[2], args[3], args[4]
    )  # self, from_name_or_id, to_name_or_id, item, arm_id


# dud main function
def main_null():
    print("This is not meant to have a main")
