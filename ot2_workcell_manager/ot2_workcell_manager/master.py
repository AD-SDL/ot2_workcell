# ROS libraries 
from atexit import unregister
from sqlite3 import Timestamp
import rclpy
from rclpy.node import Node

# OT2 Workcell Messages and Services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

# Retry API
from ot2_workcell_manager_client.retry_api import *

# OT2 Control API
from ot2_client.ot2_control_api import load_protocols_to_ot2, add_work_to_ot2

# Other Libraries
from threading import Thread, Lock
import time
from datetime import datetime
from pathlib import Path
import logging 

# Only one master node can be running at anytime, or else you will cause issues
'''
    This the Master class, which is the central command for the entire system. The purpose of the master class is to handle/provide communication among the different robots on the system. 
    To do so it is required to maintain information about every single robot running on the system which is syncronized with each robot. It is responsbile for allowing the dynamic addition
    and removal of new robots to the system easily and provide an interface for them to find other robots also connected to the system. 
''' 
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
        self.node_wait_attempts = 100  # 100 attempts before error thrown
        self.sub_list = []

        # Readability
        self.state = {"BUSY": 1, "READY": 0, "ERROR": 2, "QUEUED": 3}  # TODO: more states
        self.status = {"SUCCESS": 0, "WARNING": 2, "ERROR": 1, "FATAL": 3, "WAITING": 10}
        self.heartbeat = datetime.now() #datetime(2022,1,1,1,1,1,1) for low number
        self.heartbeat_node_list = []

        # Path setup
        path = Path()
        self.home_location = str(path.home())
        self.module_location = self.home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/"

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

        # Subscription State setup
        self.arm_state_subscriber = self.create_subscription(ArmStateUpdate, "/arm/arm_state_update", self.node_state_update_callback, 10)
        self.arm_state_subscriber
        self.OT2_state_subscriber = self.create_subscription(OT2StateUpdate, "/OT_2/ot2_state_update", self.node_state_update_callback, 10)
        self.OT2_state_subscriber 
        self.sch_state_subscriber = self.create_subscription(SchStateUpdate, "/sch/sch_state_update", self.node_state_update_callback, 10) # TODO incorporate into scheduler 
        self.sch_state_subscriber 
        self.arm_state_reset_subscriber = self.create_subscription(ArmReset, "/arm/arm_state_reset", self.state_reset_callback, 10)
        self.arm_state_reset_subscriber 
        self.OT2_state_reset_subscriber = self.create_subscription(OT2Reset, "/OT_2/ot2_state_reset", self.state_reset_callback, 10)
        self.OT2_state_reset_subscriber
        self.sch_state_reset_subscriber = self.create_subscription(SchReset, "/sch/sch_state_reset", self.state_reset_callback, 10)
        self.sch_state_reset_subscriber
        self.heartbeat_subscriber = self.create_subscription(Heartbeat, "/heartbeat/heartbeat_update", self.heartbeat_callback, 15)
        self.heartbeat_subscriber

        # Initialization Complete
        self.get_logger().info("Master initialization complete")

        # Create a thread to run check_heartbeat
        check_heartbeat_thread = Thread(target=self.check_heartbeat)
        check_heartbeat_thread.start()

    # Registers a worker with the master so modules can be distrubuted
    def handle_register(self, request, response):

        # Create response
        response = Register.Response()

        # Check type
        '''
            TODO: implement states, more types, and more state information
        '''
        if request.type == "OT_2":
            dict = {
                "type": "OT_2",
                "id": "O"
                + str(
                    self.nodes
                ),  # Can be searched along with name (each id must be unique)
                "state": self.state["READY"],  
                "name": request.name,
                "heartbeat": self.heartbeat
            }

            self.get_logger().info(
                "Trying to register ID: %s name: %s with master"
                % (dict["id"], dict["name"])
            )
        elif request.type == "arm":
            dict = { 
                "type": "arm",
                "id": "A"
                + str(
                    self.nodes
                ),  # Can be searched along with name (each id must be unique)
                "state": self.state["READY"],  
                "name": request.name,
                "heartbeat": self.heartbeat

            }

            self.get_logger().info(
                "Trying to register ID: %s name: %s with master"
                % (dict["id"], dict["name"])
            )
        elif request.type == "scheduler":
            dict = {  
                "type": "scheduler",
                "id": "sch"
                + str(
                    self.nodes
                ),  # Can be searched along with name (each id must be unique)
                "state": self.state["READY"], 
                "name": request.name,
                "heartbeat": self.heartbeat

            }

            self.get_logger().info(
                "Trying to register ID: %s name: %s with master"
                % (dict["id"], dict["name"])
            )
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
    ):

        # Lock: Entering critical section
        self.node_lock.acquire()

        # Create response
        response = Destroy.Response()

        # Find id in nodes_list
        for i in range(0, len(self.nodes_list)):
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
        dict = {"type": "-1", "name": "-1", "state": -1, "id": "-1", "heartbeat": datetime.now() } #datetime(2022,1,1,1,1,1,1)
        return dict

    # Checks to see if the node/worker is ready (registered with the master)
    '''
    def node_ready(self, args):
        entry = self.search_for_node(args[0])
        if entry["type"] == "-1":
            self.get_logger().info("Waiting on node %s" % args[0])
            return self.status["ERROR"]
        else:
            return self.status["SUCCESS"]
    '''

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
            entry.state = item["state"]
            entry.type = item["type"]
            response.node_list.append(entry)
        response.status = response.SUCCESS
        return response

    # Service to update the state of a node
    def node_state_update_callback(self, msg):
        # Find node
        entry = self.search_for_node(msg.id)
        current_state = entry['state']

        # Prevent changing state when in an error state
        if(current_state == self.state['ERROR']):
            self.get_logger().error("Can't change state, the state of node %s is already error"%msg.id)
            return # exit out of function

        # set state
        self.node_lock.acquire()
        entry['state'] = msg.state
        self.node_lock.release()

        # Print full state information 
        self.get_logger().info("----------------System State Information----------------") #TODO: will fix but using it for master state sync with nodes
        for node in self.nodes_list:
            self.get_logger().info("node ID: %s name: %s is in state: %s"%(node["id"], node["name"], node["state"]))
        self.get_logger().info("-------------------End of Information-------------------")

    '''
        Upon master deregistration it needs to alert all nodes that are still reliant on master services that it is in an errored state. In the future this doesn't mean a full shutdown
        of that node but currently as a prototype it will shutdown all nodes currently registered with the master by initiating a node_state_update to error. In order for the master 
        to properly be restarted, the current state information in self.nodes_list needs to be saved. Then upon master restart it boots from the saved file and resets the states of all
        respective nodes. Any work that was loaded in the database should still be there, but any work that was currently worked should also be restarted. 
    '''
    def master_deregistration(self):
        pass # TODO


   # Function to reset the state of the transfer handler
    def state_reset_callback(self, msg):
        self.get_logger().warning("Resetting state of id: %s..."%msg.id)

        # Find node
        entry = self.search_for_node(msg.id)

        # set state
        self.node_lock.acquire()
        entry['state'] = msg.state
        self.node_lock.release()
    
    def heartbeat_callback(self, msg):
        """heartbeat_callback

            Description: Function to receive the robot heartbeats. Updates the heartbeat time in the "nodes_list" and "hearbeat_node_list" lists.
            
            Parameters: 
                        -msg: Message, that is recived from the nodes. Only contains the robot id.
        """

        self.get_logger().info("Receiving heartbeat of id: %s..."%msg.id)
        
        # Find node
        entry = self.search_for_node(msg.id)

        # Set current time
        now = datetime.now()
 
        # set hearbeat timestamp
        self.node_lock.acquire()
        entry['heartbeat'] = now
        self.node_lock.release() 

        count = 0
        for index in range(len(self.heartbeat_node_list)):
            if self.heartbeat_node_list[index][0] == entry["id"]:
                self.heartbeat_node_list[index][1] =  now
                count+=1
        if count == 0 and entry["id"] != "-1" :
            self.heartbeat_node_list.append([entry["id"], now])

        #self.get_logger().info(str(now))
        #self.get_logger().info("heartbeat_callback --- msg id: %s, self id: %s, entry id %s"%(msg.id, self.id, entry['id']))

        


    #Scaning the heartbeat record and checkworkcell_ming the current time
    def check_heartbeat(self):
        """check_heartbeat

            Description: Triggered by the thread. 
                         Scans the heartbeat record of the each node and finds the elapsed time since the last heartbeat update. 
                         Considers the node dead if the last heartbeat update was more then 30 seconds.

        """
         # Runs every 15 seconds
        while rclpy.ok():
            
            time.sleep(15)
                     
            # Find node
            for index in range(len(self.heartbeat_node_list)):            
                #  Recive last updated heartbeat time
                last = self.heartbeat_node_list[index][1]

                last_timestamp = datetime.strptime(str(last), "%Y-%m-%d %H:%M:%S.%f")

                # Set current time
                now = datetime.now()
                current_time =datetime.strptime(str(now), "%Y-%m-%d %H:%M:%S.%f")

                is_node_alive = current_time - last_timestamp

                if(is_node_alive.seconds <= 30):
                    self.get_logger().info("Node ID: %s is alive. Last Heartbeat recived in %s seconds ago." % (self.heartbeat_node_list[index][0], is_node_alive.seconds))   
            
                elif(is_node_alive.seconds > 30):
                    entry = self.search_for_node(self.heartbeat_node_list[index][0])
                    entry['state'] = "ERROR"
                    self.get_logger().warning("Node ID: %s Heartbeat is not responding. Last Heartbeat update:  %s" %(self.heartbeat_node_list[index][0], last_timestamp))
                
            


# This is just for testing, this class can be used anywhere
def main(args=None):
    rclpy.init(args=args)
    master_controller = Master()
    
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
    master_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
