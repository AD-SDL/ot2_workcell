#To run JSON protocol use open shell format below
from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient
from rich import print, pretty, inspect

def main():
	pretty.install() 

	host_ip = '127.0.0.1'
	user = 'ubuntu'
	passwd = 'ubuntu'
 
	try:
		client = SSHClient()
		client.load_host_keys('/home/dozgulbas/.ssh/id_rsa.pub')
		client.load_system_host_keys()
		client.set_missing_host_key_policy(AutoAddPolicy())
		client.connect(host_ip, username= user, password = passwd)
		
		while True:
			try:
				cmd =input("$> ")
				if cmd == "exit": break
				stdin, stdout, stderr = client.exec_command(cmd)
				print(f'STDOUT: {stdout.read().decode("utf8")}')
				print(f'STDERR: {stderr.read().decode("utf8")}')
				print(f'Return code: {stdout.channel.recv_exit_status()}')
			except KeyboardInterrupt:
				break

		client.close()
		
	except Exception as err:
		print(str(err))

if __name__ == '__main__':
	main()