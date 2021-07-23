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


class OT2(Node):
    def __init__(self, name):

        # Node creation
        super().__init__("ot2_" + name)  # Users specifies name

        # Lock creation
        self.file_lock = Lock()  # Access to the file system
        self.run_lock = Lock()  # Only one can access this OT-2 as a resource

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
        self.load_service = self.create_service(
            LoadService, "/OT_2/%s/load" % self.id, self.load_handler
        )
        self.run_service = self.create_service(
            Run, "/OT_2/%s/run" % self.id, self.run_handler
        )
        self.get_id_service = self.create_service(
            GetId, "/OT_2/%s/get_id" % self.name, self.get_id_handler
        )
        self.send_service = self.create_service(
            SendFiles, "/OT_2/%s/send_files" % self.id, self.receive_files
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

        # TODO: create service to unload and recieve items

        # Initialization Complete
        self.get_logger().info(
            "ID: %s name: %s initialization completed" % (self.id, self.name)
        )

    # Handles send_module service calls
    def receive_files(self, request, response):

        # Get request information
        files = request.files
        files = files.split()

        # Create Response
        response = SendFiles.Response()

        # Begin reading file names
        self.get_logger().info("Reading file names")

        try:
            # Get lock
            self.file_lock.acquire()

            # Append files to work list
            # TODO: change, maybe run worker function?
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
            self.file_lock.release()
            return response

    # Handles load_module service calls
    def load_handler(self, request, response):

        # Get request information
        name = request.name
        file = self.module_location + name
        contents = request.contents

        # Create response
        response = LoadService.Response()

        # Warnings
        if path.exists(file) and request.replace == False:
            self.get_logger().warning(
                "File %s already exists on the system and replacement is false, upload terminating..."
                % name
            )
            response.status = (
                response.WARNING
            )  # Warning: in this context file already exists on system
            return response
        if request.replace == True:
            self.get_logger().warning(
                "Replacement is set to True, file %s on this system will be replace with file from master"
                % name
            )

        # Begin loading module
        self.get_logger().info("Beginning load")
        try:
            # Get lock, Entering critical section
            self.file_lock.acquire()

            # Write to file and set permissions
            f = open(file, "w")
            f.write(contents)
            f.close()
            os.chmod(file, 0o777)  # Exectuable permissions
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            response.status = response.ERROR  # Error
        else:
            self.get_logger().info("File %s loaded to OT2" % name)
            response.status = response.SUCCESS  # All good
        finally:
            # Exiting critical section
            self.file_lock.release()
            return response

    # Handles run module service calls
    def run_handler(self, request, response):
        # Acquire lock
        self.run_lock.acquire()

        # Get request information
        type = request.type
        id = request.id
        file = (
            self.module_location + request.file
        )  # Worker attaches to the well known location

        # Create response
        resposne = Run.Response()

        # Warnings / Errors
        if not id == self.id:  # Wrong ID
            self.get_logger().error(
                "Request id: %s doesn't match node id: %s" % (id, self.id)
            )
            response.status = response.ERROR
            self.run_lock.release()  # Release lock
            return response
        elif not type == "OT_2":  # Wrong type
            self.get_logger().warning(
                "The requested node type: %s doesn't match the node type of id: %s, but will still proceed"
                % (type, self.id)
            )
        elif path.exists(file) == False:  # File doesn't exist
            self.get_logger().error("File: %s doesn't exist" % (file))
            response.status = response.ERROR
            self.run_lock.release()  # Release lock
            return response

        # Get lock, entering file critical section (Can't be reading when file is still being written)
        self.file_lock.acquire()

        # import module
        self.get_logger().info("Importing module...")
        try:
            # Loading and attaching the module to the program
            spec = importlib.util.spec_from_file_location(request.file, file)
            ot2Module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ot2Module)
        except Exception as e:
            # Error
            self.get_logger().error(
                "Error occured when trying to load module %s: %r"
                % (
                    file,
                    e,
                )
            )
            response.status = response.ERROR  # Error
            self.run_lock.release()  # Release lock
            return response
        else:
            # All Good
            self.get_logger().info(
                "Module %s loaded and attached to the program" % file
            )
        finally:
            # After exiting critical section release lock
            self.file_lock.release()

        # Running the module (The function ran is work()
        self.get_logger().info("Running module...")
        try:
            # Runs well-known function work()
            ot2Module.work()
        except Exception as e:
            # Error
            self.get_logger().error(
                "Error occured when trying to run module %s: %r"
                % (
                    file,
                    e,
                )
            )
            response.status = response.ERROR  # Error
            return response
        else:
            # All good
            self.get_logger().info("Module %s successfully ran to completion" % file)
            response.status = response.SUCCESS
            return response
        finally:
            self.run_lock.release()  # Release lock no matter what

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

        # No request information

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

        # Get lock
        self.file_lock.acquire()

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
            self.file_lock.release()
                
        
        # Clear temp list
        self.temp_list.pop(0)
        return response
        

    # Overarching function. Parses through files in a job, loads and runs files
    def read_files(self):
        files = self.work_list[0]

        # thread created and files loaded and ran in worker function
        temp_thread = self.worker(files)
        self.threads_list.append(temp_thread)  # Record information

        # Setup complete for this thread
        self.get_logger().info("Setup complete for job number %s" % self.work_index)

        # TODO: Barrier?
        # New barrier for each thread (So we know when they all finish)
        self.read_files_barrier = threading.Barrier(
            2
        )  # This thread plus the main thread

        # Start thread
        for item in self.threads_list:
            item.start()

        # Waiting on finish
        self.read_from_setup_barrier.wait()

        # Join each thread
        for item in self.threads_list:
            item.join()

        # Done
        self.get_logger().info("Setup file read and run complete")
        return self.status["SUCCESS"]

    def worker(self, files):
        for file in files:
            # Debug information
            self.node_print("Running file %s" % file)

            # Load and run individual file
            # TODO place in separate function to establish barrier
            status = self.load_and_run_ot2(self, file)

            # Error checking
            if (
                status == self.master.status["ERROR"]
                or status == self.master.status["FATAL"]
            ):
                self.node_print("Thread ending...")
                break
            else:  # All Good
                self.node_print("Node moving onto next task...")

        self.node_print("thread %s done with work" % self.thread_ID)

        # Waiting on barrier to finish
        self.read_files_barrier.wait()

    # load function, runs client that calls service on this node
    def load_ot2(self, file, replacement):
        # TODO: Check for node online and select node if all on same node?

        try:
            type = "OT_2"
            id = self.id
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            return self.status["ERROR"]

        # Client setup
        load_cli = self.create_client(LoadService, "/%s/%s/load" % (type, id))
        while not load_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available, trying again...")

        # Client ready, read contents of file
        try:
            f = open(self.module_location + file, "r")
            contents = f.read()
            f.close()
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            return self.status["ERROR"]  # Error

        self.get_logger().info("File %s read complete" % name)  # Contents of file read

        # Create Request
        load_request = LoadService.Request()
        load_request.name = file  # File path: insert file name, the file path even though the same is given to the client to set up
        load_request.contents = contents  # File string contents
        load_request.replace = replacement  # If the file exists do we overwrite it?

        # Call service to load module
        future = load_cli.call_async(load_request)
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
                # Error handling
                if response.status == response.ERROR:
                    self.get_logger().error(
                        "Error occured in loading at %s for file %s" % (id, name)
                    )
                    return self.status["ERROR"]  # Error
                elif response.status == response.WARNING:
                    self.get_logger().warning(
                        "Warning: File %s already exists on system %s" % (name, id)
                    )
                    return self.status["WARNING"]  # Warning
                else:
                    self.get_logger().info("Load succeeded")
                    return self.status["SUCCESS"]  # All good

    # run function, runs client that calls service on this node
    def run_ot2(self, file):

        # TODO: Check for node online and select node if all on same node?

        try:
            type = "OT_2"
            id = self.id
        except Exception as e:
            self.get_logger().error("Error occured: %r" % (e,))
            return self.status["ERROR"]

        # Client setup
        run_cli = self.create_client(
            Run, "/%s/%s/run" % (type, id)
        )  # format of service is /{type}/{id}/{service name}
        while not run_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available, trying again...")

        # Create a request
        req = Run.Request()
        req.type = type
        req.id = id
        req.file = file

        # Call service
        future = run_cli.call_async(req)
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
                # Error checking
                if response.status == response.ERROR:
                    self.get_logger().error(
                        "Error occured when running file %s at id: %s" % (name, id)
                    )
                    return self.status["ERROR"]  # Error
                else:
                    self.get_logger().info("Module run succeeded")
                    return self.status["SUCCESS"]  # All good

    # Function to segway to main function call
    def _load(self, args):
        return self.load(args[0], args[1], args[2])  # File, id (self), replacement

    # Function to segway to main function call
    def _run(self, args):
        return self.run(args[0], args[1])  # File, id (self)

    # load_and_run funtion using retry
    def load_and_run_ot2(self, file):
        # Load module
        id = self.id
        args = []
        args.append(file)
        args.append(id)
        args.append(True)  # Auto update
        status = retry(
            self, self._load, 1, 0, args
        )  # Calling retry function with 1 attempt, just want output information

        # Status check
        if status == self.status["FATAL"]:
            self.get_logger().fatal(
                "Major error occured when attempting to run function"
            )
            return self.status["FATAL"]  # Fatal error
        elif status == self.status["ERROR"]:
            self.get_logger().error(
                "Retry function stopped, either due to too many attempts or a bad status returned"
            )
            return self.status["ERROR"]  # Error
        elif status == self.status["WARNING"]:
            self.get_logger().warning(
                "Warning thrown by retry function during execution, but ran to completion"
            )
            # Continue
        else:
            self.get_logger().info("Function ran to completion")
            # Continue

        # Run the module
        args = []
        args.append(file)
        args.append(id)
        status = retry(
            self, self._run, 1, 0, args
        )  # Calling retry function with 1 attempt to get output messages

        # Status check
        if status == self.status["FATAL"]:
            self.get_logger().fatal(
                "Major error occured when attempting to run function"
            )
            return self.status["FATAL"]  # Fatal error
        elif status == self.status["ERROR"]:
            self.get_logger().error(
                "Retry function stopped, either due to too many attempts or a bad status returned"
            )
            return self.status["ERROR"]  # Error
        elif status == self.status["WARNING"]:
            self.get_logger().warning(
                "Warning thrown by retry function during execution, but ran to completion"
            )
            return self.status["WARNING"]  # Warnning thrown
        else:
            self.get_logger().info("Function ran to completion")
            return self.status["SUCCESS"]  # All Good


# TODO: DELETE
def work(ot2node, name):
    args = []
    if name == "bob":
        args.append(ot2node)
        args.append("bob")
        args.append("alex")
        args.append("10")
        args.append("army")
        status = retry(ot2node, _load_transfer, 20, 4, args)
    if name == "alex":
        args.append(ot2node)
        args.append("bob")
        args.append("alex")
        args.append("10")
        args.append("army")
        status = retry(ot2node, _load_transfer, 20, 4, args)


def setup_read_files(self):
    status = self.read_files(self)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("need 1 arguments")
        sys.exit(1)
    name = str(sys.argv[1])

    ot2node = OT2(name)
    
    rclpy.spin(ot2node)

    # Setup args and end
    args = []
    args.append(ot2node)  # Self
    status = retry(ot2node, _deregister_node, 10, 1.5, args)  # TODO: handle status
    spin_thread.join()
    ot2node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
