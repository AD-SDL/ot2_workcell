# ot2_workcell

The `ot2_workcell` repo consists of a collection of ROS2 packages which work in tandem to operate the Opentrons OT2-based liquid handling operations of Argonne National Laboratory's Rapid Prototyping Lab (ANL RPL).
This so far includes a workcell manager, OT2 controller, plate-handling robotic arm controller, each of the previous component's respective clients, and a set of common interfaces, with the ability to add new components on an as-needed basis.

This workcell will eventually support:
* Dynamic, dependency-driven and task-based resource scheduling
* Resource reservations
* Intelligent error handling and recovery
* Closed-loop AI experimentation and "Autonomous Discovery" functionality

# Directory Tree

* **arm_controller/**
	* **arm_controller/**
		* armManager.py (Manages the state of the arm)
		* armTransferHandler.py (Manages transfer service)
	* **launch/**
		* arm_bringup.launch.py (Launches the arm)

* **arm_client/**
	* **arm_client/**
		* transfer_api.py (An api for other OT2s to use to transfer items)
		* publish_arm_state.py (api to publish its state to a manager)

* **ot2_workcell_manager/**
	* **ot2_workcell_manager/**
		* master.py (Controls the state of the system)

* **master_api/**
	* **master_api/**
		* register_api.py (registration to the master api)
		* retry_api.py (api to allow users to run a function in an isolated manner)
		* worker_info_api.py (api to allow workers to retrieve other worker info from the master)

* **ot2_controller/**
	* **ot2_controller/**
		* protocol_manager.py (removes items from the queue in manager)

* **ot2_client/**
	* **ot2_client/**
		* load_run_api.py (TODO)
		* publish_ot2_state_api.py (api to publish the state of the ot2 to manager)

* **workcell_interfaces/**
	* **srv/**
		* Destroy.srv
		* GetId.srv
		* GetNextTransfer.srv
		* GetNodeInfo.srv
		* GetNodeList.srv
		* LoadService.srv
		* LoadTransfer.srv
		* Register.srv
		* Run.srv
	* **msg/**
		* NodeEntry.msg
		* ArmStateUpdate.msg
		* CompletedTransfer.msg
		* OT2StateUpdate.msg

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
1. `git clone https://github.com/AD-SDL/ros2tests`
1. `cd ~/ot2_ws`
1. `rosdep update && rosdep install -i --from-path src --rosdistro foxy -y`
1. `colcon build`
1. `source install/setup.bash`

## Launching

TODO: Update this to match new architecture

**Workcell Manager**
1. source ~/ros2tests/install/setup.bash
1. ros2 run ot2_workcell_manager master

**Arm**
1. source ~/ros2tests/install/setup.bash
1. ros2 launch arm_controller arm_bringup.launch.py

**Opentrons OT-2 bob**
1. source ~/ros2tests/install/setup.bash
1. ros2 run rostalker2 OT2 bob

**Opentrons OT-2 alex**
1. source ~/ros2tests/install/setup.bash
1. ros2 run rostalker2 OT2 alex

This will cause nodes to be registered with master and start a transfer process as well as a OT-2 procedure. In the future this won't be able to run conncurentlly as use of a arm will block whatever called it.

# Diagrams for different components
TODO
