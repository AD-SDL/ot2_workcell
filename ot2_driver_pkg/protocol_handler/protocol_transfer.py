from paramiko import SSHClient, AutoAddPolicy
import paramiko
from scp import SCPClient
from rich import print, pretty, inspect
from pathlib import Path
 
def transfer(local_path, user, host_ip):
	pretty.install() 

	# Get home path
	path = Path()
	home_location = str(path.home())
	try:
		client = SSHClient()

		#LOAD HOST KEYS
		key_path = home_location + "/.ssh/id_rsa.pub"
		client.load_host_keys(key_path)
		client.load_system_host_keys()
		client.set_missing_host_key_policy(AutoAddPolicy())
		#client.connect(host_ip, username= user, password = passwd)
		client.connect(host_ip, username=user)

		#Setup SCP transfer
		scp = SCPClient(client.get_transport())
		scp.put(local_path, recursive=True, remote_path='/tmp') #TODO: OT2 have a folder called data where the protocols are suppose to be stored 
	except paramiko.AuthenticationException:
		print("Authentication failed, please verify your credentials: %s")
		return 1 # error
	except paramiko.SSHException as sshException:
		print("Unable to establish SSH connection: %s" % sshException)
		return 1 # error
	except paramiko.BadHostKeyException as badHostKeyException:
		print("Unable to verify server's host key: %s" % badHostKeyException)
		return 1 # error
	except Exception as e: 
		print("Error occured in transfer: %r"%(e,))
		return 1 # error
	else:
		return 0 # all good 
	finally:
		client.close()
		scp.close()

def main_null():
	print("This function is not meant to have a main function")