import zmq
import time

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.connect("tcp://127.0.0.1:1234")
#sock.subscribe("") # Subscribe to all topics

print("Starting receiver loop ...")
while True:
    msg = "Is_Tip"
    sock.send_string(msg)
    msg = sock.recv_string()
    print("Received string: %s ..." % msg)
    

sock.close