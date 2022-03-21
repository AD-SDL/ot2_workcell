import zmq
import time
from multiprocessing.connection import Listener
import sys
from database.database_functions import *
from database import connect
import protocol_transfer 
from protocol_parser import *
from zeroMQ_OT2 import *



def handler(Protocol_ID):
    path, protocol = pull_protocol(Protocol_ID)  
    print("Protocol saved into " + path + "directory")
    #protocol_transfer.traprotocol_transfernsfer(path)
    protocol_path = "/data" + protocol
    msg_error, msg_output = send_message_to_OT2("python3 "+ protocol_path)
    return msg_output, msg_error

def send_message_to_OT2(message):

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect("tcp://192.168.1.81:8085")

    print("Starting loop...")
    while True:
        sock.send_string(message)
        print("Sent string: %s ..." % message)
        time.sleep(1)
        msg = sock.recv_string()
        msg = msg.split('@')
        msg_output, msg_error = msg[0], msg[1]
        if msg_output != None:
            print("Client recived the output message. Sending message to ROS Master")
            return msg_error, msg_output
    sock.close()

if __name__ == "__main__":
    filename = "/ot2_workcell/protocol_handling/protocol.py"
    # new_name = protocol_parser(filename)
    # protocol_ID = insert_protocol(new_name, "OT2_1")
    protocol_ID = 7
    handler(protocol_ID)
 