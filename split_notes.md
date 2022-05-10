# Split Notes 
Notes on how to split repo into smaller individual modules (move robot specific drivers out of repo). Dependency graph is at `deps.png`

## Current Package Split 
Want to split packages away from the ROS packages (we only want ROS packages in the ROS repo). The packages listed could be abstracted so that they can be easily interchanged and treated as drivers. 

* OT2 (database packages might be split)
    * `protocol_handler` Interactions with the OT2 specifics doesn't need to be in the ROS package 
    * `zeroMQ_OT2` 
* scheduler 
    * `schedulerManager.py` the `batch scheduling` function. The actual scheduling component is independent from ROS and can be moved out so that you can easily attached new functions with no knowledge of ROS.
* arm (arm transfer specifics)
    * `armTransferManager.py` site at which transfers are called
* database 
    * `database` database packages could be moved out and abstracted
