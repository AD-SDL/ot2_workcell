# ROS Libraries
import rclpy
from rclpy.node import Node

# Time Library 
import time

# ROS services and messages
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# Function to determine if a node is ready 
def node_ready(self, name_or_id):
    entry = get_node_info(self, name_or_id)
    if entry["type"] == "-1":
        self.get_logger().info("Waiting on node %s" % name_or_id)
        return self.status["WAITING"]
    else:
        return self.status["SUCCESS"]

# Function to determine if a node is ready 
def _node_ready(self, args):
    return node_ready(args[0], args[1])

# Get master manager entry about the specific node
def get_node_info(self, name_or_id):

    # Create request
    request = GetNodeInfo.Request()
    request.name_or_id = name_or_id

    # Client setup
    get_node_info_cli = self.create_client(
        GetNodeInfo, "get_node_info"
    )
    
    # Wait for service to start
    while not get_node_info_cli.wait_for_service(
        timeout_sec=2.0
    ):
        self.get_logger().info("Service not available, trying again...")

    # Call service to get node info
    future = get_node_info_cli.call_async(request)
    self.get_logger().info("Waiting on node info for %s" % name_or_id)

    # Waiting on future
    while future.done() == False:
        time.sleep(1)  # 1 second timeout
    if future.done():
        entry = {"type": "-1"}
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Error occured %r" % (e,))
            return entry  # Error
        else:
            self.get_logger().info("Node info for %s recieved" % name_or_id)
            entry["type"] = response.entry.type
            entry["name"] = response.entry.name
            entry["state"] = response.entry.state
            entry["id"] = response.entry.id
            return entry  # All Good


# Middleman to segway from retry functions or others to the get_node_info
def _get_node_info(args):
    return get_node_info(
        args[0], args[1]
    )  # TODO: Current retry_api do not support this type of output


# Service call to retrieve information about all the workers registered with master
def get_node_list(self):  # TODO: testing

    # Create a request
    request = GetNodeList.Request()

    # Client setup
    get_node_list_cli = self.create_client(GetNodeList, "get_node_list")
    while not get_node_list_cli.wait_for_service(
        timeout_sec=2.0
    ):  # Wait for service to start
        self.get_logger().info("Service not available, trying again...")

    # Call Service
    future = get_node_list_cli.call_async(request)
    self.get_logger().info("Waiting on node list")

    # Waiting on future
    while future.done() == False:
        time.sleep(1)  # 1 second timeout
    nodes_list = []
    if future.done():
        try:
            response = future.result()

            if (
                response.status == response.ERROR or response.status == response.FATAL
            ):  # get_node_list service doesn't support this right now
                raise Exception
        except Exception as e:
            self.get_logger().error("Error occured %r" % (e,))
            nodes_list.append({"type": "-1"})  # Error, type of item 0 is '-1'
        else:
            self.get_logger().info("Node list recieved")
            for msg in response.node_list:
                entry = {}
                entry["id"] = msg.id
                entry["name"] = msg.name
                entry["state"] = msg.state
                entry["type"] = msg.type
                nodes_list.append(entry)
    return nodes_list


# Middleman to segway from retry functions or others to the get_node_list
def _get_node_list(args):
    return get_node_list(
        args[0]
    )  # Self TODO: Current retry_api do not support this type of output


def main_null():
    print("This is not meant to have a main function")
