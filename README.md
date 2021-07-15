# ros2tests
Basic tests with ros2, repo for when learning ROS2, and repo for running basic rostalker2 files during the intial prototyping stage.

# Rostalker2 
Rostalker2 is a project to connect 3 different moving parts in an automated biology lab a master, Opentron-2 robot, and robotic arm, with added equipment later on. A goal of rostalker2 is to set up a network between these different robotics parts using ROS 2 Foxy and have them able to syncronize each other and be fault tolerant. Another goal is to allow a user to run whatever they want concurrently on multiple different machines this goal is called the monolithic model. Another goal is to be able to run biology experiments in a HPC manner, where the user just specifies tasks and their dependencies and the system will automatically schedule the experiment, this theoritcallly should increase performance and efficiency dubbed the workflow model. Currently rostalker2 only supports the monolithic model, future plans will begin to add in the workflow model. 

# Rostalker2 Files
**armtalker/**   
&nbsp;&nbsp;  **armtalker/**  
&nbsp;&nbsp;&nbsp;&nbsp;    armManager.py (Manages the state of the arm)  
&nbsp;&nbsp;&nbsp;&nbsp;    armTransferHandler.py (Manages transfer service)  
&nbsp;&nbsp;  **launch/**  
&nbsp;&nbsp;&nbsp;&nbsp;    arm_bringup.launch.py (Launches the arm)  
  
**armtalker_api/**  
&nbsp;&nbsp;  **armtalker_api/**  
&nbsp;&nbsp;&nbsp;&nbsp;    transfer_api.py (An api for other OT2s to use to transfer items)  
&nbsp;&nbsp;&nbsp;&nbsp;    publish_arm_state_api.py (api to publish its state to a manager)  

**mastertalker/**  
&nbsp;&nbsp;  **mastertalker/**  
&nbsp;&nbsp;&nbsp;&nbsp;    master.py (Controls the state of the system)  
&nbsp;&nbsp;&nbsp;&nbsp;    worker_thread.py (No longer needed)  

**master_api/**  
&nbsp;&nbsp;  **master_api/**  
&nbsp;&nbsp;&nbsp;&nbsp;    register_api.py (registration to the master api)  
&nbsp;&nbsp;&nbsp;&nbsp;    retry_api.py (api to allow users to run a function in an isolated manner)  
&nbsp;&nbsp;&nbsp;&nbsp;    worker_info_api.py (api to allow workers to retrieve other worker info from the master)  
    
**ot2talker/**  
&nbsp;&nbsp;  **ot2talker/**  
&nbsp;&nbsp;&nbsp;&nbsp;    protocol_manager.py (removes items from the queue in manager)  

**ot2talker_api/**  
&nbsp;&nbsp;  **ot2talker_api/**  
&nbsp;&nbsp;&nbsp;&nbsp;    load_run_api.py (TODO)  
&nbsp;&nbsp;&nbsp;&nbsp;    publish_ot2_state_api.py (api to publish the state of the ot2 to manager)  

**rostlalker2/**  
&nbsp;&nbsp;  **rostalker2/**  
&nbsp;&nbsp;&nbsp;&nbsp;    ot2class.py (Contains the state of the OT2 class planned to split it up into 3 different files)  
 
**rostalker2interface/**  
&nbsp;&nbsp;  **srv/**  
&nbsp;&nbsp;&nbsp;&nbsp;    Destroy.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    GetId.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    GetNextTransfer.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    GetNodeInfo.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    GetNodeList.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    LoadService.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    LoadTransfer.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    Register.srv  
&nbsp;&nbsp;&nbsp;&nbsp;    Run.srv  
&nbsp;&nbsp;  **msg/**  
&nbsp;&nbsp;&nbsp;&nbsp;    NodeEntry.msg  
&nbsp;&nbsp;&nbsp;&nbsp;    ArmStateUpdate.msg  
&nbsp;&nbsp;&nbsp;&nbsp;    CompletedTransfer.msg  
&nbsp;&nbsp;&nbsp;&nbsp;    OT2StateUpdate.msg  

# Install 
This is for ubuntu 20.04 running ROS2 foxy  
1. cd (Must be in home directory)
2. source /opt/ros/foxy/setup.bash
3. git clone https://github.com/urd00m/ros2tests
4. cd ros2tests
5. colcon build 
6. source install/setup.bash

# Launching 
**Master**  
1. source ~/ros2tests/install/setup.bash
2. ros2 run mastertalker master

**Arm**  
1. source ~/ros2tests/install/setup.bash
2. ros2 launch armtalker arm_bringup.launch.py

**Opentron-2s**
1. source ~/ros2tests/install/setup.bash
2. ros2 run rostalker2 OT2 bob

**Opentron-2s**
1. source ~/ros2tests/install/setup.bash
2. ros2 run rostalker2 OT2 alex

This will cause nodes to be registered with master and start a transfer process as well as a OT-2 procedure. In the future this won't be able to run conncurentlly as use of a arm will block whatever called it. 

# Diagrams for different components
TODO

