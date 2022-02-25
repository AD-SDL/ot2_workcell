from subprocess import PIPE, run


def execute_command(command):
    print("Executing the given command on the OT2")
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print(result.stdout)
    print(result.stderr)
    return result

