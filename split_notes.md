# Split Notes 
Notes on how to split repo into smaller individual modules (move robot specific drivers out of repo). Dependency graph is at `deps.png`

## Current Package Split 
Want to split packages away from the ROS packages (we only want ROS packages in the ROS repo). The packages listed could be abstracted so that they can be easily interchanged and treated as drivers. 

* OT2 (database packages might be split) (**Note:** this package requires a database abstraction)
    * `protocol_handler` Interactions with the OT2 specifics doesn't need to be in the ROS package 
    * `zeroMQ_OT2` 
* scheduler 
    * `schedulerManager.py` the `batch scheduling` function. The actual scheduling component is independent from ROS and can be moved out so that you can easily attached new functions with no knowledge of ROS.
* arm (arm transfer specifics)
    * `armTransferManager.py` site at which transfers are called
* database 
    * `database` database packages could be moved out and abstracted

Current system structure
1. User Interface  
2. ROS  
    a. TODO: chart for the ROS structure
3. robot drivers (pf400, ot2)  
4. database driver  

# OT2 abstraction requirments
Moving OT2 abstractions out of ROS network. 

## Naming Conventions
Package **must** be named `ot2_driver_pkg`, and file must be named `ot2_driver`. **No other package** will be imported into the ROS network, and thus invisible to the ROS layer. 

## Functions needed 
* load_protocol(protocol_path, robot_id)
    * Loads the protocol to the ot2_driver (knows what protocol code is, current driver does this via database)
    * Inserts in error handling into the protocol
* run_protocol(protocol_id, username, ip, port)
    * Runs the given protocol id on the specified internal RPI4
    * TODO: get that internal RPI4 specifications from config file not passed via arguments