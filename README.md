# ros2tests
Basic tests with ros2, repo for when learning ROS2, and repo for running basic rostalker2 files during the intial prototyping stage.

# Rostalker2
TODO: get descriptions set up
**armClass.py**  
**armManager.py**  
**armTransferHandler.py**  
**load_run_api.py**  
**master.py**  
**ot2Class.py**  
**register_functions.py**  
**retry_functions.py**  
**transfer_api.py**  
**worker_info_api.py**  
**worker_thread.py**  

### TODO more information about rostalker2

# Install 
This is for ubuntu 20.04 running ROS2 foxy
0. cd (Must be in home directory)
1. source /opt/ros/foxy/setup.bash
2. git clone https://github.com/urd00m/ros2tests
3. cd ros2tests
4. colcon build 
5. source install/setup.bash

# Launching 
**Computer 1**
1. source ~/ros2tests/install/setup.bash
2. ros2 run rostalker2 master

**Computer 2**
1. source ~/ros2tests/install.setup.bash
2. ros2 run rostalker2 OT2

Currently this will load a basic hello_world module onto the OT2 node and run it, on the terminal that you ran OT2 you should see the words hello world pop up 

# Diagrams for different components
TODO

# TODOs
1) look at how OT-2 code is uploaded   
2) get pipeline feature setup   
3) better input from user  
4) topic pub from different robots   
5) attempt to run sim   
