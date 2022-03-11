from subprocess import PIPE, run
import time
import zmq
from stream_camera import camera_client


def listen():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)

    sock.connect("tcp://IP:1234")
    sock.subscribe("") # Subscribe to all topics

    print("Starting receiver loop ...")
    while True:
        msg = sock.recv_string()
        
        if msg == "tip":
            is_tip = camera_client(msg)
        if msg == "True":    
            print(msg)
        elif msg == "False":
            print(msg)    
            #sock.send_strting(is_tip)

    
    sock.close() 


if __name__ == "__main__":
    listen()