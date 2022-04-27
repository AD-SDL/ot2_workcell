from subprocess import PIPE, run


def execute_command(command):
    print("Executing the given command on the OT2")
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print(result.stdout)
    print(result.stderr)
    return result, result.stdout, result.stderr

def main_null():
    print("This function is not meant to have a main function") 