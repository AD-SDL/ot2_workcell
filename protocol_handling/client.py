import zmq
import time

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:1234")

print("Starting loop...")
while True:
    msg = "/usr/local/bin/python3.9 /Path/To/protocol_handling/protocol.py"
    sock.send_string(msg)
    print("Sent string: %s ..." % msg)
    time.sleep(1)
    
sock.close()

 