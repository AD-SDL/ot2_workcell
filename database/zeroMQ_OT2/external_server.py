import zmq
import time
from stream_camera import camera_client

if __name__ == "__main__":
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

def main_null():
    print("This function is not meant to have a main function")