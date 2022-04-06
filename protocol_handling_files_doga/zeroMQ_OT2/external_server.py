import zmq
import time
from stream_camera import camera_client

ctx = zmq.Context()
sock = ctx.socket(zmq.REP)
sock.bind("tcp://*:1234")

print("Starting loop...")
i = 1
while True:
    ms = sock.recv_string()
    print(ms)
    if ms == "Is_Tip":
        error_message = camera_client(ms)
        sock.send_string(error_message)
        print("Sent string: %s ..." % error_message)
    i += 1
    time.sleep(1)
    
   

sock.close()