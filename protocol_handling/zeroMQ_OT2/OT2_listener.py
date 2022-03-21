from subprocess import PIPE, run
import time
import zmq


def listen():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind("tcp://*:8085")
    print("Starting loop...")
    i = 1
    while True:
        msg = sock.recv_string()
        print(msg)
        i += 1
        time.sleep(1)
        if msg != None:
            msg_output, msg_error = execute_command(msg)
            sock.send_string(msg_output + '@' + msg_error)


    sock.close() 

def execute_command(command):
    print("Executing the given command on the OT2")
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print(result.stdout)
    print(result.stderr)
    print (result)
    return str(result.stdout), str(result.stderr)

if __name__ == "__main__":
    listen()