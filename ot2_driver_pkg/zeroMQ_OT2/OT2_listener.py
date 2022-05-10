from subprocess import PIPE, run
import time
import zmq


def listen():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind("tcp://*:8085")
    print("Starting loop for OT2 internal listener...")
    i = 1
    while True:
        msg = sock.recv_string()
        print(msg)
        i += 1
        time.sleep(1)
        if msg != None:
            msg_output, msg_error, msg_returncode = execute_command(msg)
            sock.send_string(msg_output + '@' + msg_error + '@' + str(msg_returncode))

 
    sock.close() 

def execute_command(command):
    print("<---------------- Executing the protocol on the OT2 --------------->")    
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print("Return code: ",  result.returncode)
    print("Error Message:\n" + result.stderr)
    print("Output Message:\n" + result.stdout)
    return result.stdout, result.stderr, result.returncode

def main_null():
    print("This function is not meant to have a main function")

if __name__ == "__main__":
    listen()