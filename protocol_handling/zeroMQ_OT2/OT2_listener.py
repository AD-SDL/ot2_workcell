from subprocess import PIPE, run
import time
import zmq


def listen():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect("tcp://127.0.0.1:1234")
    sock.subscribe("") # Subscribe to all topics

    print("Starting receiver loop ...")
    while True:
        msg = sock.recv_string()
        result, msg_error, msg_output = execute_command.execute_command(msg)
        sock.send_strting(msg_error, msg_output)

    
    sock.close() 

def execute_command(command):
    print("Executing the given command on the OT2")
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print(result.stdout)
    print(result.stderr)
    return result, result.stdout, result.stderr

if __name__ == "__main__":
    listen()