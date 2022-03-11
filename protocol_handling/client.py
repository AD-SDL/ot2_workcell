import zmq
import time
from multiprocessing.connection import Listener
import sys
from database.database_functions import *
from database import connect
import protocol_transfer 
from protocol_parser import *
from zeroMQ_OT2 import *
from vision_pipette.stream_camera import camera_client



def handler(Protocol_ID):
    path, protocol = pull_protocol(Protocol_ID)  
    print("Protocol saved into " + path + "directory")
    protocol_transfer.transfer(path)
    #START EXTERNAL_listener
    msg_error, msg_output = send_message_to_OT2("python3 "+ protocol)
    return msg_error, msg_output

def send_message_to_OT2(message):

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind("tcp://127.0.0.1:1234")

    print("Starting loop...")
    while True:
        msg = message
        sock.send_string(msg)
        print("Sent string: %s ..." % msg)
        time.sleep(1)
        result, msg_error, msg_output = sock.recv_string()
        if msg_error != None or msg_output != None:
            return msg_error, msg_output
    sock.close()

if __name__ == "__main__":
    # filename = "/Users/dozgulbas/Desktop/OT2/ot2_workcell/protocol_handling/protocol.py"
    # new_name = protocol_parser(filename)
    # protocol_ID = insert_protocol(new_name, "OT2_1")
    protocol_ID = 1
    handler(protocol_ID)
 