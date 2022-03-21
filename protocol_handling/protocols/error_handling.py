import os
import sys
import pickle
import socket
import struct
import sys
import zmq
import time
import cv2
import numpy as np
import opentrons.execute
import opentrons.simulate

'''
Contains error handling functions to recover from failures using the external camera to detect the errors. 

'''
def client_external(message):

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect("tcp://127.0.0.1:1234")
    #sock.subscribe("") # Subscribe to all topics

    print("Starting message loop ...")
    while True:
        #msg = "Is_Tip"
        if message == "Is_Tip":
            sock.send_string(message)
            msg = sock.recv_string()
            print("Received message: %s " % msg)
            if msg != None:
                return msg
            
 

def pick_another_tip(p20, protocol):
    #protocol.pause()
    print("!!!Tip is not attached. Picking up a new tip ...!!!")
    p20.drop_tip()
    p20.pick_up_tip()

def return_tip(p20, protocol):
    protocol.pause()
    print("!!!Placing the tip back to its' location ...!!!")
    p20.return_tip()
    p20.pick_up_tip()

def drop_tip(p20, protocol):
    protocol.pause()
    print("!!!Tip is not droped!!!")
    p20.drop_tip()

def is_tip_attached(p20, protocol):
    message = "Is_Tip"
    message = client_external(message)

    # if  message == "Impropriate Tip":
    #     return_tip(p20, protocol)

    if message.strip() == "False":
        pick_another_tip(p20, protocol)
    else:
        print("Tip is attached properly")
 
def is_tip_dropped(p20, protocol):
    message = "Is_Tip"
    message = client_external(message)

    # if  message == "Impropriate Tip":
    #     return_tip(p20, protocol)

    if  message.strip() == "False":
        print("Tip was dropped. No error!")
    else:
        print("Tip is still attached")
        drop_tip(p20, protocol)

def repeat_aspirate():
    '''TODO
    - Check if the plate is in the correct location (use camera)
    - Check if we have liquid in the cells (use camera)
    - If the labware is not the problem repeat aspirate 
    - If the problem is with the labware, fix labware
    '''
    pass

def is_aspirate(p20, protocol):
    #TODO: Return error detection message from Rory's code
    repeat_aspirate()
    pass

def is_dispense(p20,protocol):
    #TODO: Return error detection message from Rory's code
    pass

if __name__ == "__main__":
    message = "tip"
    client_external(message)