from paramiko import SSHClient, AutoAddPolicy
import paramiko
from scp import SCPClient
from rich import print, pretty, inspect

def transfer(local_path):
	pretty.install()
	host_ip = '0.0.0.0'
	user = 'nerra'
	passwd = 'password'
	try:
		client = SSHClient()
		#LOAD HOST KEYS
		#client.load_host_keys('~/.ssh/known_hosts')
		client.load_host_keys('~/.ssh/id_rsa.pub')
		client.load_system_host_keys()
		client.set_missing_host_key_policy(AutoAddPolicy())
		client.connect(host_ip, username= user, password = passwd)
		#Setup SCP transfer
		scp = SCPClient(client.get_transport())
		scp.put(local_path, recursive=True, remote_path='/data/')

	
	except paramiko.AuthenticationException:
		print("Authentication failed, please verify your credentials: %s")
	
	except paramiko.SSHException as sshException:
		print("Unable to establish SSH connection: %s" % sshException)
	
	except paramiko.BadHostKeyException as badHostKeyException:
		print("Unable to verify server's host key: %s" % badHostKeyException)
	
	except paramiko.SCPException as err:
		print(err)	
	
	finally:
		client.close()
		scp.close()

def main_null():
	print("This function is not meant to have a main function")