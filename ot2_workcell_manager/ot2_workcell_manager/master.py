import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *
from pathlib import Path
from ot2_workcell_manager_client.retry_api import *

# Transfer api import
from arm_client.transfer_api import *
from arm_client.transfer_api import _load_transfer

# Only one master node can be running at anytime, or else you will cause issues
class Master(Node):
    def __init__(self):
        # Node creation
        super().__init__(
            "master_node"
        )  # TODO: Add in the ability to have multiple masters (maybe?)

        # Lock setup
        self.node_lock = (
            Lock()
        )  # This lock controls access to the self.nodes and self.nodes_list structure

        # Registration setup
        self.nodes = 0  # Total nodes registered
        self.nodes_list = []  # Information about all nodes registered: type, id, state
        self.node_wait_timeout = 2  # 2 seconds
        self.node_wait_attempts = 10  # 10 attempts before error thrown

        # Thread setup
        self.files_for_threads = (
            []
        )  # Maintains list of all the files each thread saved in threads_list needs to run
        self.threads_list = []  # Maintains all the thread objects

        # Readability
        self.states = {"BUSY": "1", "READY": "0"}  # TODO: more states
        self.status = {"SUCCESS": 0, "WARNING": 2, "ERROR": 1, "FATAL": 3}

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ros2tests/OT2_Modules/"

        # Basic informaton
        self.id = "M-1"  # Ultimate position, before 0
        self.type = "master"  # Of type master
        self.name = "master"  # name is master

        # Service setup
        self.register_service = self.create_service(
            Register, "register", self.handle_register
        )  # registration service
        self.destroy_service_master = self.create_service(
            Destroy, "destroy", self.handle_destroy_worker
        )  # Destroy worker service
        self.get_node_info_service = self.create_service(
            GetNodeInfo, "get_node_info", self.handle_get_node_info
        )  # Request is a name_or_id and returns all the information master has about that node
        self.get_node_list_service = self.create_service(
            GetNodeList, "get_node_list", self.handle_get_node_list
        )  # Blank request returns a list of all the nodes the master knows about

        # Client setup
        # TODO: see if any clients can be setup here

        # Initialization Complete
        self.get_logger().info("Master initialization complete")

    # Registers a worker with the master so modules can be distrubuted
    def handle_register(self, request, response):

        # Create response
        response = Register.Response()

        # Check type
        if request.type == "OT_2":
            dict = {
                "type": "OT_2",
                "id": "O"
                + str(
                    self.nodes
                ),  # Can be searched along with name (each id must be unique)
                "state": self.states["READY"],  # TODO: implement states
                "name": request.name,
            }
            self.get_logger().info(
                "Trying to register ID: %s name: %s with master"
                % (dict["id"], dict["name"])
            )
        elif request.type == "arm":
            dict = {  # TODO: add more features that the master keeps about the node
                "type": "arm",
                "id": "A"
                + str(
                    self.nodes
                ),  # Can be searched along with name (each id must be unique)
                "state": self.states["READY"],  # TODO: implement states
                "name": request.name,
            }
            self.get_logger().info(
                "Trying to register ID: %s name: %s with master"
                % (dict["id"], dict["name"])
            )
        # TODO: more types
        else:
            self.get_logger().error(
                "type %s not supported at this moment" % request.type
            )
            response.status = response.ERROR  # Error
            return response

        # Create response
        response.status = response.SUCCESS
        response.id = dict["id"]  # Send back the ID to the worker

        # Lock: Critical section entry
        self.node_lock.acquire()

        # Update Node information: done last
        self.nodes += 1
        self.nodes_list.append(dict)

        # Release lock and exit
        self.node_lock.release()
        self.get_logger().info(
            "Registration of ID: %s name: %s complete" % (dict["id"], dict["name"])
        )
        return response

    # Removes node information upon service call
    def handle_destroy_worker(
        self, request, response
    ):  # TODO make it request name as well

        # Lock: Entering critical section
        self.node_lock.acquire()

        # Create response
        response = Destroy.Response()

        # Find id in nodes_list
        for i in range(0, self.nodes):
            dict = self.nodes_list[i]
            if dict["id"] == request.id and dict["type"] == request.type:
                self.nodes_list.pop(i)  # Remove from list
                self.get_logger().info(
                    "Removed id: %s of type: %s name: %s from nodes_list"
                    % (dict["id"], dict["type"], dict["name"])
                )
                response.status = response.SUCCESS
                self.node_lock.release()
                return response
            # Error checking
            elif dict["id"] == request.id and not dict["type"] == request.type:
                self.nodes_list.pop(i)  # Remove from list
                self.get_logger().info(
                    "Warning! id: %s name: %s doesn't match type in service request, type in request: %s, actual type: %s"
                    % (dict["id"], dict["name"], request.type, dict["type"])
                )
                response.status = response.WARNING
                self.node_lock.release()
                return response

        # No id found in nodes_list
        self.get_logger().error(
            "Unable to find id: %s of type: %s" % (request.id, request.type)
        )
        response.status = response.ERROR
        self.node_lock.release()
        return response

    # Helper function to search nodes_list by id
    def search_for_node(self, id_or_name):
        # get lock entering critical section
        self.node_lock.acquire()

        for dict in self.nodes_list:
            if dict["id"] == id_or_name or dict["name"] == id_or_name:
                # release lock and exit
                self.node_lock.release()
                return dict

        # leaving critical section
        self.node_lock.release()

        # Not found
        dict = {"type": "-1", "name": "-1", "state": "-1", "id": "-1"}
        return dict

    # Checks to see if the node/worker is ready (registered with the master)
    def node_ready(self, args):
        entry = self.search_for_node(args[0])
        if entry["type"] == "-1":
            self.get_logger().info("Waiting on node %s" % args[0])
            return self.status["ERROR"]
        else:
            return self.status["SUCCESS"]

    # Creates client that sends files to worker OT-2 to create threads
    def send_files(self, id, files):  # self, id of robot, and files of current job

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
            target_node = self.search_for_node(id)  # See if id robot exists

            # Error checking
            if target_node["type"] == "-1":  # No such node
                self.get_logger().error("id: %s doesn't exist" % id)
                return self.status["ERROR"]

            type = target_node["type"]  # These will be needed to acess the service
            id = target_node["id"]

        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            return self.status["ERROR"]

        # create client that calls file handler service on OT-2 module

        # Client setup
        send_cli = self.create_client(
            SendFiles, "/%s/%s/send_files" % (type, id)
        )  # format of service is /{type}/{id}/{service name}
        while not send_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available, trying again...")

        # Client ready
        # TODO: replacement parameter?
        # Create a request
        send_request = SendFiles.Request()
        # send_request.numFiles = len(files) # number of files to be sent to worker node
        send_request.files = files  # string of file names list

        # Call Service to load module
        future = send_cli.call_async(send_request)

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
                    self.get_logger().info("Files loaded")
                    return self.status["SUCCESS"]  # All good

    # Reads from a setup file to run a number of files on a specified robot
    def read_from_setup(self, file):  # TODO: deadlock detection algorithm
        # Read from setup file and distrubute to worker threads
        # Read number of threads
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
            args = []
            args.append(name_or_id)
            status = retry(
                self,
                self.node_ready,
                self.node_wait_attempts,
                self.node_wait_timeout,
                args,
            )  # retry function
            if status == self.status["ERROR"] or status == self.status["FATAL"]:
                self.get_logger().error(
                    "Unable to find node %s" % name_or_id
                )  # Node isn't registered
                return self.status["ERROR"]
            else:
                entry = self.search_for_node(
                    name_or_id
                )  # get information about that node
                id = entry["id"]
                self.get_logger().info("Node %s found" % name_or_id)  # Found

            # Get files for the worker
            try:
                files = f.readline()

                self.files_for_threads.append(files)  # should be the same index
            except Exception as e:
                self.get_logger().error("Reading from setup error: %r" % (e,))
                return self.status["ERROR"]  # Error

            # files sent to worker OT-2 to become threads
            self.send_files(id, files)

            # Setup complete for this thread
            self.get_logger().info("Setup complete for %s" % name_or_id)

        self.get_logger().info("Setup file read and run complete")
        return self.status["SUCCESS"]

    # Handles get node info service call
    def handle_get_node_info(self, request, response):
        # Create a response
        response = GetNodeInfo.Response()

        # Get request
        name_or_id = request.name_or_id

        # TODO: DELETE Debug
        self.get_logger().info("Node info request for %s" % name_or_id)

        # Get request
        entry = self.search_for_node(name_or_id)

        # Edit response
        response.entry.id = entry["id"]
        response.entry.name = entry["name"]
        response.entry.state = entry["state"]
        response.entry.type = entry["type"]  # Differ error checking back to caller
        response.status = response.SUCCESS  # all good

        # return response
        return response

    # Hanldes get the whole node list service call
    def handle_get_node_list(self, request, response):  # TODO testing
        # Create response
        response = GetNodeList.Response()

        # Edit response
        for item in self.nodes_list:
            entry = NodeEntry()
            entry.id = item["id"]
            entry.name = item["name"]
            entry.sate = item["state"]
            entry.type = item["type"]
            response.node_list.append(entry)
        response.status = response.SUCCESS
        return response



# TODO: Add a deregister master, so if the master disconnects or deregisters the workers can start waiting for a new master

# This is just for testing, this class can be used anywhere
def main(args=None):
    rclpy.init(args=args)
    master_controller = Master()

    # Create a thread to run setup_thread
    spin_thread = Thread(target=master_controller.read_from_setup, args=("setup",))
    spin_thread.start()

    # master.read_from_setup("setup")

    # 	status = master.load("module_test.py", "O0",  False)
    # 	status2 = master.run("module_test.py", "O0")
    # 	status = master.load_and_run("module_test.py", "O0")

    # Spin
    try:
        rclpy.spin(master_controller)
    except Exception as e:
        master_controller.get_logger().fatal(
            "rclpy.spin failed, system in volatile state %r" % (e,)
        )
    except:
        master_controller.get_logger().fatal("Terminating...")

    # End
    spin_thread.join()
    master_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
