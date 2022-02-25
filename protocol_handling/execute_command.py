from subprocess import PIPE, run


def execute_command(command):
    result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
    print(result.stdout, result.stderr)
    #print(result.stderr)
    return result

