# ot2_workcell

The `ot2_workcell` repo consists of a collection of ROS2 packages which work in tandem to operate the Opentrons OT2-based liquid handling operations of Argonne National Laboratory's Rapid Prototyping Lab (ANL RPL).
This so far includes a workcell manager, OT2 controller, plate-handling robotic arm controller, each of the previous component's respective clients, and a set of common interfaces, with the ability to add new components on an as-needed basis.

This workcell will eventually support:
* Dynamic, dependency-driven and task-based resource scheduling
* Resource reservations
* Intelligent error handling and recovery
* Closed-loop AI experimentation and "Autonomous Discovery" functionality

# Development

## Install ROS2 Foxy

* [On Ubuntu, Mac, or Windows](https://docs.ros.org/en/foxy/Installation.html)
* [On a Raspberry Pi 4](https://roboticsbackend.com/install-ros2-on-raspberry-pi/)

## Install the Packages

This is assuming an Ubuntu 20.04 environment with ROS Foxy installed.

1. `cd ~`
2. `source /opt/ros/foxy/setup.bash`
3. `mkdir -p ~/ot2_ws/src`
4. `cd ~/ot2_ws/src`
5. `git clone https://github.com/AD-SDL/ot2_workcell.git`
6. `cd ot2_workcell && git clone https://github.com/AD-SDL/ot2_driver_pkg.git && git clone -b ros_driver https://github.com/AD-SDL/PF400_cobot.git` Bring in OT2 driver  
**Note** you will need to change the folder name `PF400_cobot` to `arm_driver_pkg`  
7. `cd ~/ot2_ws`
8. `rosdep update && rosdep install -i --from-path src --rosdistro foxy -y`
9. `sudo apt install python3-colcon-common-extensions`
10. `colcon build`
11. `source install/setup.bash`

## Database Setup 
1. In `database/protocol_handler/protocol_handling_client.py` in the function`send_message_to_OT2(...)` sock.connect(...) needs to be changed to your IP and whatever port you want 
2. To Install Mysql Database Server.
* Note: Installation process is included for testing purposes with the local setups. Eventually, Mysql server will be runnnig on the servers located in Argonne National Laboratory and this process will not be necessary. 
* `sudo apt-get update && sudo apt-get upgrade`
* `sudo apt install mysql-server`
* `mysql --version` ->> Check if the installation was successful
3. Configure Mysql Server for Remote Connections
* `sudo nano /etc/mysql/mysql.conf.d/mysqld.cnf` ->> Edit "bind-address = 127.0.0.1". Use "0.0.0.0" for all remote connections (not suggested for security reasons) or use the IP address of the machine that will be used for remote connections 
* `sudo systemctl restart mysql`
* `sudo mysql -u root` ->> Log in to MySQL Server
* `CREATE USER 'username'@'remote_server_ip' IDENTIFIED BY 'password';` 
* `GRANT CREATE, ALTER, DROP, INSERT, UPDATE, DELETE, SELECT, REFERENCES, RELOAD on *.* TO 'username'@'remote_server_ip' WITH GRANT OPTION;`
* `FLUSH PRIVILEGES;`
4. Create a config.py in the home directory and include the below lines
* #Database variables
* DBNAME = "DATABASENAME"
* DBUSER = "USERNAME"
* DBPASSWD = "USERPASSWORD"
* DBHOST = "HOST_NAME or HOST_IP_ADDRESS"
5. For testing on the actual OT2! In `database/protocol_handler/protocol_parser.py` in the function `protocol_parser(...)` the commented line `new_file.write("import error_handling\n")` needs to be uncommented 
6. In `database/protocol_handler/protocol_transfer.py` in the function `transfer(...)` the `host_ip` and `user` need to be changed to match the database you have 
7. Protocols need to be added to the `/data` folder, to change this in `database/protocol_handler/protocol_transfer.py` in the function `transfer(...)` the line `scp.put(local_path, recursive=True, remote_path='/tmp')` the remote path `/tmp` needs to be changed to `/data`. You also need to change in `database/protocol_handler/protocol_handling_client.py` in the function `handler(...)` the line `msg_error, msg_output, msg_errorcode = send_message_to_OT2("python3 "+ "/tmp/" + protocol.split("/")[-1])` the `/tmp/` needs to be changed to `/data`

## Launching OT2_workcell

**Workcell Manager**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 run ot2_workcell_manager master`

**Arm**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 launch arm_controller arm_bringup.launch.py`

**Opentrons OT-2 bob**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 launch ot2_controller ot2_bob_bringup.launch.py`

**Opentrons OT-2 alex**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 launch ot2_controller ot2_alex_bringup.launch.py`

**Scheduler Manager**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 run scheduler_controller scheduler_manager`

**Scheduler Work Adder**
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 run scheduler_controller scheduler_work_adder`

**OT2 Client**  
This is for each OT2 that you plan on recieving jobs on and must be **run on the OT2**.
1. `source ~/ot2_ws/install/setup.bash`
2. `python3 ~/ot2_ws/src/ot2_workcell/ot2_driver_pkg/zeroMQ_OT2/OT2_listener.py`

This will cause nodes to be registered with master and you can insert workflow files via the `Scheduler Work Adder` which will prompt you for workflow files. It will automatically schedule 
that workflow to available OT2s.

## Test Bench for Deadlock Detection 
1. `source ~/ot2_ws/install/setup.bash`
2. `python3 ~/ot2_ws/src/ot2_workcell/scheduler_client/scheduler_client/test_bench.py`

This will run all of the deadlock detection tests.

## Testing for Reading Workflow Files
1. `source ~/ot2_ws/install/setup.bash`
2. `ros2 run scheduler_client json_scheduler_reader`  

This will read the `workflow.json` file in the OT2_modules directory and print it to the screen 

## Workflow file format 

Workflow files are json files and must be formatted as such. The first section is the `blocks` section. You specify a list of blocks each block has the following information, 
1. `block-name`, the unique name of the block, the scheduler showed throw an error if there are duplicate names
2. `tasks`, a string with all the different task that need to be run separated by spaces 
3. `dependencies`, a string with all the dependencies off that block that are separated by spaces  

**Format** 
```
'blocks': [ 	{'block-name':'[Your Block Name]', 'tasks':'[protocol1 protocol2 protocol3 ...]', 'dependencies':'[block_name_1 block_name_2 ...]'},
		{'block-name':'[Your Block Name]', 'tasks':'[protocol1 protocol2 protocol3 ...]', 'dependencies':'[block_name_1 block_name_2 ...]'},
		{'block-name':'[Your Block Name]', 'tasks':'[protocol1 protocol2 protocol3 ...]', 'dependencies':'[block_name_1 block_name_2 ...]'},
		{...}
	   ]
```

After the `blocks` section you have the `meta-data` (dictionary) section which contains 
1. `author`, the creator of the workflow file
2. `email`, the email of the creator 
3. `description`, describes what the workflow file does  

**Format**
```
"meta-data": {
		"author": "[Author Name]",
		"email": "[Author Email]", 
		"description": "[Description of the Workflow File]"
	     }    
```  
Example workflow file is in the `OT2_Modules/workflow.json`.

# Diagrams for different components
![Diagram of state](https://raw.githubusercontent.com/AD-SDL/ot2_workcell/master/Diagrams/stateot2_diagram.png)
![Diagram of ROS to Database](https://raw.githubusercontent.com/AD-SDL/ot2_workcell/master/Diagrams/protocol_handling_diagram.png)
![Proposal Diagram](https://raw.githubusercontent.com/AD-SDL/ot2_workcell/master/Diagrams/Proposal.png)


## Useful Commands 
* `colcon graph --dot | dot -Tpng -o deps.png` Generates a dependency graph image for the packages
