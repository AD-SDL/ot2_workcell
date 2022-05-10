import zmq
import time

if __name__ == '__main__':
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect("tcp://127.0.0.1:8085")
    #sock.subscribe("") # Subscribe to all topics

    print("Starting receiver loop ...")
    while True:
        msg = "Is_Tip"
        sock.send_string(msg)
        msg = sock.recv_string()
        print("Received string: %s ..." % msg)
        

    sock.close
 
def main_null():
    print("This function is not meant to have a main function")