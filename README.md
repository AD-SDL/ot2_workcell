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
1. `source /opt/ros/foxy/setup.bash`
1. `mkdir -p ~/ot2_ws/src`
1. `cd ~/ot2_ws/src`
1. `git clone https://github.com/AD-SDL/ot2_workcell.git`
1. `cd ~/ot2_ws`
1. `rosdep update && rosdep install -i --from-path src --rosdistro foxy -y`
1. `sudo apt install python3-colcon-common-extensions`
3. `colcon build`
4. `source install/setup.bash`

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

This will cause nodes to be registered with master and start a transfer process as well as a OT-2 procedure. In the future this won't be able to run conncurentlly as use of a arm will block whatever called it.

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
'blocks': [ {'block-name':'[Your Block Name]', 'tasks':'[protocol1 protocol2 protocol3 ...]', 'dependencies':'[block_name_1 block_name_2 ...]'},
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
![Diagram of ROS to Database](https://raw.githubusercontent.com/AD-SDL/ot2_workcell/scheduler/Diagrams/protocol_handling_diagram.png)
