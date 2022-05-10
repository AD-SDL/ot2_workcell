# ROS Libraries
import rclpy
from rclpy.node import Node

# Database functions
from database.database_functions import *
from database.database_functions import insert_protocol
from protocol_handler.protocol_parser import *
from protocol_handler.protocol_parser import protocol_parser
from protocol_handler.protocol_handling_client import *
from protocol_handler.protocol_handling_client import handler

# Path libraries
from pathlib import Path

'''
    This function uploads a protocol to the OT2 driver (however, the driver wishes to distribute it)

    Input: Protocol Path, and a robot ID (that ROS will use later)
    Output: A protocol_id for it to use to retrieve the just uploaded file
'''
def load_protocol(protocol_path, robot_id):

    # insert error handling 
    protocol_new_name = protocol_parser(protocol_path)

    # insert protocol into database
    path = Path()
    home_location = str(path.home()) #TODO: switch to a less ROS dependent path (maybe in /tmp)
    protocol_module_location = home_location + "/ot2_ws/src/ot2_workcell/Protocol_Modules/" # Get Protocol_Module location
    protocol_id = insert_protocol(protocol_module_location + protocol_new_name, robot_id)

    # Return protocol ID
    return protocol_id

'''
    This function runs a given protocol (however, the driver wishes to load and then run and return output)
    
    Input: protocol_id, username, ip, and port (username, ip, and port are the username, ip, and port of internal OT2)
    TODO: switch username, ip, and port to a config file ROS should not have to feed it in as input
    Output: error_msg, output_msg, status_code (0-SUCCESS, 1-ERROR, 2-WARNING, 3-FATAL)
'''
def run_protocol(protocol_id, username, ip, port):
    
    return handler(protocol_id, username, ip, port)

def main_null():
    print("This is not meant to have a main function")

if __name__=='__main__':
    main_null()